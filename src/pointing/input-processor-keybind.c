/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/sys/dlist.h>
#include <drivers/behavior.h>
#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>
#include <zmk/behavior_queue.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdlib.h>
#include <zmk/pointing/zip_keybind.h>
#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND_STUDIO_RPC)
#include <zephyr/settings/settings.h>
#endif

#define DT_DRV_COMPAT zmk_input_processor_keybind

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum zip_keybind_key_state {
    ZIP_KEY_NONE = 0,
    ZIP_KEY_RIGHT = 1,
    ZIP_KEY_LEFT = 2,
    ZIP_KEY_HORIZONTAL = BIT(ZIP_KEY_LEFT) | BIT(ZIP_KEY_RIGHT),
    ZIP_KEY_DOWN = 3,
    ZIP_KEY_UP = 4,
    ZIP_KEY_VERTICAL = BIT(ZIP_KEY_UP) | BIT(ZIP_KEY_DOWN),
};

struct zip_keybind_config {
    uint8_t index;
    uint8_t mode;

    const char *name; // devicetree node name, exposed over the gesture RPC

    bool track_remainders;
    bool continuous_key_press;
    struct zmk_behavior_binding *bindings;       // mutable at runtime (live binding swap)
    const struct zmk_behavior_binding *default_bindings; // ROM copy for reset
    uint8_t num_bindings;
    uint32_t tap_ms;
    uint32_t wait_ms;
    uint32_t tick;

    int32_t threshold;
    int32_t max_threshold;

    int32_t max_pending_activations;
};

struct zip_keybind_data {
    int32_t delta_x;
    int32_t delta_y;

    int32_t last_delta_x;
    int32_t last_delta_y;

    int32_t max_delta;
    uint8_t device_index;
    enum zip_keybind_key_state state;

    // Runtime-tunable params. Copied from config at init, then mutable at
    // runtime via the custom Studio RPC (see studio/keybind_custom_handler.c).
    // All processing reads these (data->) instead of the const config (cfg->),
    // so changes take effect live without reflashing.
    uint32_t tick;
    uint32_t wait_ms;
    uint32_t tap_ms;
    int32_t threshold;
    int32_t max_threshold;

    const struct device *dev;
    struct k_work_delayable press_work;
};

static inline bool has_pending_movement(const struct zip_keybind_data *data,
                                        const struct zip_keybind_config *cfg) {
    ARG_UNUSED(cfg);
    return abs(data->delta_x) >= data->tick || abs(data->delta_y) >= data->tick;
}

static uint32_t approx_hypot(uint32_t a, uint32_t b) {
    if (a < b) {
        unsigned int tmp = a;
        a = b;
        b = tmp;
    }
    return a + ((b * 3) >> 3);
}

static void keybind_handle_raw(struct zip_keybind_data *data,
                               const struct zip_keybind_config *cfg) {

    data->delta_x = CLAMP(data->delta_x + data->last_delta_x, -data->max_delta, data->max_delta);
    data->delta_y = CLAMP(data->delta_y + data->last_delta_y, -data->max_delta, data->max_delta);

    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

static void keybind_handle_4way(struct zip_keybind_data *data,
                                const struct zip_keybind_config *cfg) {
    int32_t movement = approx_hypot(abs(data->last_delta_x), abs(data->last_delta_y));
    if (abs(data->last_delta_x) > abs(data->last_delta_y)) {
        if (data->last_delta_x < 0)
            movement = -movement;

        data->delta_x = CLAMP(data->delta_x + movement, -data->max_delta, data->max_delta);
    } else {
        if (data->last_delta_y < 0)
            movement = -movement;

        data->delta_y = CLAMP(data->delta_y + movement, -data->max_delta, data->max_delta);
    }

    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

static void keybind_handle_8way(struct zip_keybind_data *data,
                                const struct zip_keybind_config *cfg) {
    int32_t dx = data->last_delta_x;
    int32_t dy = data->last_delta_y;
    int32_t adx = abs(dx);
    int32_t ady = abs(dy);

    if (dy == 0) {
        data->delta_x = CLAMP(data->delta_x + dx, -data->max_delta, data->max_delta);
    } else if (dx == 0) {
        data->delta_y = CLAMP(data->delta_y + dy, -data->max_delta, data->max_delta);
    } else {
        int32_t movement = approx_hypot(adx, ady);

        // check tan for 22.5° sector, which is approx. 5/12
        if (5 * adx > 12 * ady) {
            // horizontal movement (±22.5°)
            data->delta_x = CLAMP(data->delta_x + (dx < 0 ? -movement : movement), -data->max_delta,
                                  data->max_delta);
        } else if (5 * ady > 12 * adx) {
            // vertival movement (±22.5°)
            data->delta_y = CLAMP(data->delta_y + (dy < 0 ? -movement : movement), -data->max_delta,
                                  data->max_delta);
        } else {
            // diagonals
            int32_t delta_x = (dx < 0) ? -movement : movement;
            int32_t delta_y = (dy < 0) ? -movement : movement;
            data->delta_x = CLAMP(data->delta_x + delta_x, -data->max_delta, data->max_delta);
            data->delta_y = CLAMP(data->delta_y + delta_y, -data->max_delta, data->max_delta);
        }
    }

    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

static int zip_keybind_handle_event(const struct device *dev, struct input_event *event,
                                    uint32_t param1, uint32_t param2,
                                    struct zmk_input_processor_state *state) {
    const struct zip_keybind_config *cfg = dev->config;
    struct zip_keybind_data *data = dev->data;
    int32_t value = event->value;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // cutoff small or very large movements
    if (data->threshold > abs(value) || abs(value) > data->max_threshold)
        return ZMK_INPUT_PROC_STOP;

    // Accumulate movement
    if (event->code == INPUT_REL_X) {
        data->last_delta_x = value;
    } else if (event->code == INPUT_REL_Y) {
        data->last_delta_y = value;
    } else {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // wait until full movement readed
    if (!event->sync)
        return ZMK_INPUT_PROC_STOP;

    int32_t movement = approx_hypot(abs(data->last_delta_x), abs(data->last_delta_y));

    if (cfg->mode == 0) {
        keybind_handle_raw(data, cfg);
    } else if (cfg->mode == 1) {
        keybind_handle_4way(data, cfg);
    } else if (cfg->mode == 2) {
        keybind_handle_8way(data, cfg);
    }

    LOG_DBG("dev: %d dx: %d dy: %d tick: %d", state->input_device_index, data->delta_x,
            data->delta_y, data->tick);

    if (has_pending_movement(data, cfg)) {
        data->device_index = state->input_device_index;

        if (!k_work_delayable_is_pending(&data->press_work)) {
            k_work_schedule(&data->press_work, K_NO_WAIT);
        }
    }

    return ZMK_INPUT_PROC_STOP;
}

static inline uint32_t get_position(const struct zip_keybind_data *data,
                                    const struct zip_keybind_config *cfg) {
    return ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(data->device_index, cfg->index);
}

static int invoke_binding(struct zip_keybind_data *data, const struct zip_keybind_config *cfg,
                          int idx, bool pressed) {
    if ((pressed && data->state & BIT(idx)) || (!pressed && (data->state & BIT(idx) == 0)))
        return 0; // already pressed or released, skip

    // normalize key flag to array index position
    int bindingId = idx - 1;

    struct zmk_behavior_binding_event behavior_event = {
        .position = get_position(data, cfg),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_invoke_binding(&cfg->bindings[bindingId], behavior_event, pressed);

    if (ret == 0) {
        if (pressed)
            data->state |= BIT(idx);
        else
            data->state &= ~BIT(idx);
    }

    LOG_DBG("trigger binding: bh: %s 0x%02x 0x%02x pressed: %s state %X",
            cfg->bindings[bindingId].behavior_dev, cfg->bindings[bindingId].param1,
            cfg->bindings[bindingId].param2, pressed ? "true" : "false", data->state);

    return ret;
}

static inline void check_and_release_key(struct zip_keybind_data *data,
                                         const struct zip_keybind_config *cfg, int idx) {
    if (data->state & BIT(idx)) {
        invoke_binding(data, cfg, idx, false);
    }
}

static void press_work_cb(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct zip_keybind_data *data = CONTAINER_OF(d_work, struct zip_keybind_data, press_work);
    const struct device *dev = data->dev;
    const struct zip_keybind_config *cfg = dev->config;

    bool has_queued_movement = has_pending_movement(data, cfg);

    if (data->state != ZIP_KEY_NONE && (!cfg->continuous_key_press || !has_queued_movement)) {
        // release keys, naive implementation
        check_and_release_key(data, cfg, ZIP_KEY_LEFT);
        check_and_release_key(data, cfg, ZIP_KEY_RIGHT);
        check_and_release_key(data, cfg, ZIP_KEY_UP);
        check_and_release_key(data, cfg, ZIP_KEY_DOWN);

        // wait after release
        k_sleep(K_MSEC(data->wait_ms));
    }

    if (has_queued_movement) {
        int idx = ZIP_KEY_NONE;
        int idy = ZIP_KEY_NONE;

        if (abs(data->delta_x) >= data->tick) {
            if (data->delta_x > 0) { // RIGHT
                idx = ZIP_KEY_RIGHT;
                data->delta_x -= data->tick;

                check_and_release_key(data, cfg, ZIP_KEY_LEFT);
            } else { // LEFT
                idx = ZIP_KEY_LEFT;
                data->delta_x += data->tick;

                data->state |= ZIP_KEY_LEFT;

                check_and_release_key(data, cfg, ZIP_KEY_RIGHT);
            }
        } else if (cfg->continuous_key_press) {
            // Check and release horizontal keys since no movement on this axis detected
            check_and_release_key(data, cfg, ZIP_KEY_LEFT);
            check_and_release_key(data, cfg, ZIP_KEY_RIGHT);
        }

        if (abs(data->delta_y) >= data->tick) {
            if (data->delta_y > 0) { // UP
                idy = ZIP_KEY_UP;
                data->delta_y -= data->tick;

                check_and_release_key(data, cfg, ZIP_KEY_DOWN);
            } else { // DOWN
                idy = ZIP_KEY_DOWN;
                data->delta_y += data->tick;

                check_and_release_key(data, cfg, ZIP_KEY_UP);
            }
        } else if (cfg->continuous_key_press) {
            // Check and release vertical keys since no movement on this axis detected
            check_and_release_key(data, cfg, ZIP_KEY_UP);
            check_and_release_key(data, cfg, ZIP_KEY_DOWN);
        }

        if (idx != ZIP_KEY_NONE)
            invoke_binding(data, cfg, idx, true);

        if (idy != ZIP_KEY_NONE)
            invoke_binding(data, cfg, idy, true);

        k_work_schedule(&data->press_work, K_MSEC(data->tap_ms));
    }

    // clear remainders after all movement processed
    if (!cfg->track_remainders) {
        data->delta_x = 0;
        data->delta_y = 0;
    }
}

static struct zmk_input_processor_driver_api zip_keybind_driver_api = {
    .handle_event = zip_keybind_handle_event,
};

static int zip_keybind_init(const struct device *dev) {
    struct zip_keybind_data *data = dev->data;
    const struct zip_keybind_config *cfg = dev->config;

    data->dev = dev;

    // Seed the runtime-tunable params from the devicetree config. After this,
    // all processing reads data-> values, which the custom RPC can change live.
    data->tick = cfg->tick;
    data->wait_ms = cfg->wait_ms;
    data->tap_ms = cfg->tap_ms;
    data->threshold = cfg->threshold;
    data->max_threshold = cfg->max_threshold;

    data->max_delta = cfg->max_pending_activations * data->tick;

    k_work_init_delayable(&data->press_work, press_work_cb);
    return 0;
}

#define TRANSFORMED_BINDINGS(n)                                                                    \
    {LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}

#define ZIP_KEYBIND_INST(n)                                                                        \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) >= 4, "bindings should have at least 4 elements");  \
    static struct zip_keybind_data zip_keybind_data_##n = {                                        \
        .delta_x = 0,                                                                              \
        .delta_y = 0,                                                                              \
        .state = ZIP_KEY_NONE,                                                                     \
    };                                                                                             \
    static struct zmk_behavior_binding zip_keybind_config_bindings_##n[] =                         \
        TRANSFORMED_BINDINGS(n);                                                                   \
    static const struct zmk_behavior_binding zip_keybind_default_bindings_##n[] =                  \
        TRANSFORMED_BINDINGS(n);                                                                   \
    static struct zip_keybind_config zip_keybind_config_##n = {                                    \
        .index = n,                                                                                \
        .name = DT_NODE_FULL_NAME(DT_DRV_INST(n)),                                                 \
        .mode = DT_INST_PROP_OR(n, mode, 0),                                                       \
        .bindings = zip_keybind_config_bindings_##n,                                               \
        .default_bindings = zip_keybind_default_bindings_##n,                                      \
        .num_bindings = DT_INST_PROP_LEN(n, bindings),                                             \
        .track_remainders = DT_INST_PROP_OR(n, track_remainders, false),                           \
        .continuous_key_press = DT_INST_PROP_OR(n, continuous_key_press, false),                   \
        .tap_ms = DT_INST_PROP_OR(n, tap_ms, 20),                                                  \
        .wait_ms = DT_INST_PROP_OR(n, wait_ms, 0),                                                 \
        .tick = DT_INST_PROP_OR(n, tick, 10),                                                      \
        .threshold = DT_INST_PROP_OR(n, threshold, 1),                                             \
        .max_threshold = DT_INST_PROP_OR(n, max_threshold, 200),                                   \
        .max_pending_activations = DT_INST_PROP_OR(n, max_pending_activations, 5)};                \
    DEVICE_DT_INST_DEFINE(n, &zip_keybind_init, NULL, &zip_keybind_data_##n,                       \
                          &zip_keybind_config_##n, POST_KERNEL,                                    \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &zip_keybind_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_KEYBIND_INST)

#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND_STUDIO_RPC)

/* Runtime registry + tuning API used by the custom Studio RPC handler
 * (src/studio/gesture_handler.c). Declared in include/zmk/pointing/zip_keybind.h.
 * All getters/setters operate on the RAM-resident params in zip_keybind_data,
 * so changes take effect immediately without reflashing. */

#define ZIP_KEYBIND_DEV(n) DEVICE_DT_INST_GET(n),

static const struct device *const zip_keybind_devs[] = {
    DT_INST_FOREACH_STATUS_OKAY(ZIP_KEYBIND_DEV)};

int zip_keybind_get_count(void) { return ARRAY_SIZE(zip_keybind_devs); }

int zip_keybind_get_info(uint32_t id, struct zip_keybind_info *out) {
    if (id >= ARRAY_SIZE(zip_keybind_devs) || out == NULL) {
        return -EINVAL;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    const struct zip_keybind_data *data = dev->data;

    out->name = cfg->name;
    out->tick = data->tick;
    out->wait_ms = data->wait_ms;
    out->tap_ms = data->tap_ms;
    out->threshold = data->threshold;
    out->max_threshold = data->max_threshold;
    return 0;
}

int zip_keybind_set_param(uint32_t id, enum zip_keybind_param param, int32_t value) {
    if (id >= ARRAY_SIZE(zip_keybind_devs)) {
        return -EINVAL;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    struct zip_keybind_data *data = dev->data;

    switch (param) {
    case ZIP_KEYBIND_PARAM_TICK:
        // tick is the divisor for accumulated movement; 0 would never fire.
        data->tick = (uint32_t)MAX(value, 1);
        data->max_delta = cfg->max_pending_activations * data->tick;
        break;
    case ZIP_KEYBIND_PARAM_WAIT_MS:
        data->wait_ms = (uint32_t)MAX(value, 0);
        break;
    case ZIP_KEYBIND_PARAM_TAP_MS:
        data->tap_ms = (uint32_t)MAX(value, 0);
        break;
    case ZIP_KEYBIND_PARAM_THRESHOLD:
        data->threshold = value;
        break;
    case ZIP_KEYBIND_PARAM_MAX_THRESHOLD:
        data->max_threshold = value;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

// --- NVS persistence (Zephyr settings) ----------------------------------
// Saved values survive reboot. Keys are "zkb/<id>/<param>" holding an int32.
// On boot ZMK's settings_load() replays them via zkb_settings_set, which
// re-applies into RAM after the devicetree defaults were seeded at init.

#define ZIP_KEYBIND_SETTINGS_SUBTREE "zkb"
#define ZIP_KEYBIND_PARAM_COUNT 5

static void zip_keybind_settings_key(char *buf, size_t len, uint32_t id, uint32_t param) {
    snprintf(buf, len, ZIP_KEYBIND_SETTINGS_SUBTREE "/%u/%u", id, param);
}

int zip_keybind_save_param(uint32_t id, enum zip_keybind_param param, int32_t value) {
    char key[24];
    zip_keybind_settings_key(key, sizeof(key), id, (uint32_t)param);
    return settings_save_one(key, &value, sizeof(value));
}

// --- NVS persistence for direction bindings (Zephyr settings) ---------------
// Keys are "gkb/<id>/<dir>" holding a zip_keybind_binding_nvs struct.
// Name string is saved (not behavior_id) so load is robust after reboot
// without depending on local_id re-mapping.

#define ZIP_KEYBIND_GKB_SUBTREE "gkb"
#define ZIP_KEYBIND_GKB_NAME_MAX 24

struct zip_keybind_binding_nvs {
    char name[ZIP_KEYBIND_GKB_NAME_MAX];
    int32_t param1;
    int32_t param2;
};

static void zip_keybind_gkb_key(char *buf, size_t len, uint32_t id, uint32_t dir) {
    snprintf(buf, len, ZIP_KEYBIND_GKB_SUBTREE "/%u/%u", id, dir);
}

int zip_keybind_get_binding(uint32_t id, uint32_t dir, uint16_t *behavior_id,
                            int32_t *param1, int32_t *param2) {
    if (id >= ARRAY_SIZE(zip_keybind_devs) || dir >= ZIP_KEYBIND_DIR_COUNT) {
        return -EINVAL;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    const struct zmk_behavior_binding *b = &cfg->bindings[dir];
    if (behavior_id) {
        *behavior_id = b->behavior_dev ? zmk_behavior_get_local_id(b->behavior_dev) : 0;
    }
    if (param1) { *param1 = (int32_t)b->param1; }
    if (param2) { *param2 = (int32_t)b->param2; }
    return 0;
}

int zip_keybind_set_binding(uint32_t id, uint32_t dir, uint16_t behavior_id,
                            int32_t param1, int32_t param2) {
    if (id >= ARRAY_SIZE(zip_keybind_devs) || dir >= ZIP_KEYBIND_DIR_COUNT) {
        return -EINVAL;
    }
    const char *name = zmk_behavior_find_behavior_name_from_local_id(behavior_id);
    if (!name) {
        return -ENODEV;
    }
    const struct device *bdev = zmk_behavior_get_binding(name);
    if (!bdev) {
        return -ENODEV;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    // cfg->bindings[] is a non-const array; only cfg itself is const-qualified.
    // Writing through the mutable pointer stored in cfg->bindings is valid.
    cfg->bindings[dir].behavior_dev = bdev->name; // device->name is static storage, always valid
    cfg->bindings[dir].param1 = (uint32_t)param1;
    cfg->bindings[dir].param2 = (uint32_t)param2;
    return 0;
}

int zip_keybind_save_binding(uint32_t id, uint32_t dir) {
    if (id >= ARRAY_SIZE(zip_keybind_devs) || dir >= ZIP_KEYBIND_DIR_COUNT) {
        return -EINVAL;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    const struct zmk_behavior_binding *b = &cfg->bindings[dir];
    struct zip_keybind_binding_nvs rec = {0};
    if (b->behavior_dev) {
        strncpy(rec.name, b->behavior_dev, sizeof(rec.name) - 1);
    }
    rec.param1 = (int32_t)b->param1;
    rec.param2 = (int32_t)b->param2;
    char key[24];
    zip_keybind_gkb_key(key, sizeof(key), id, dir);
    return settings_save_one(key, &rec, sizeof(rec));
}

// settings_load() callback. name is the subtree-relative part "<id>/<dir>".
static int zip_keybind_gkb_set(const char *name, size_t len, settings_read_cb read_cb,
                               void *cb_arg) {
    const char *slash = strchr(name, '/');
    if (!slash) { return -ENOENT; }
    uint32_t id = (uint32_t)strtoul(name, NULL, 10);
    uint32_t dir = (uint32_t)strtoul(slash + 1, NULL, 10);
    struct zip_keybind_binding_nvs rec;
    if (len != sizeof(rec)) { return -EINVAL; }
    ssize_t rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) { return (int)rc; }
    rec.name[sizeof(rec.name) - 1] = '\0';
    if (id >= ARRAY_SIZE(zip_keybind_devs) || dir >= ZIP_KEYBIND_DIR_COUNT) {
        return 0; // 範囲外は無視(壊れたNVSで落ちない)
    }
    const struct device *bdev = zmk_behavior_get_binding(rec.name);
    if (!bdev) {
        return 0; // 解決不能なら適用せずdevicetreeデフォルトのまま(堅牢degrade)
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    cfg->bindings[dir].behavior_dev = bdev->name;
    cfg->bindings[dir].param1 = (uint32_t)rec.param1;
    cfg->bindings[dir].param2 = (uint32_t)rec.param2;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(zip_keybind_gkb, ZIP_KEYBIND_GKB_SUBTREE, NULL,
                               zip_keybind_gkb_set, NULL, NULL);

int zip_keybind_reset(uint32_t id) {
    if (id >= ARRAY_SIZE(zip_keybind_devs)) {
        return -EINVAL;
    }
    const struct device *dev = zip_keybind_devs[id];
    const struct zip_keybind_config *cfg = dev->config;
    struct zip_keybind_data *data = dev->data;

    data->tick = cfg->tick;
    data->wait_ms = cfg->wait_ms;
    data->tap_ms = cfg->tap_ms;
    data->threshold = cfg->threshold;
    data->max_threshold = cfg->max_threshold;
    data->max_delta = cfg->max_pending_activations * data->tick;

    // Drop any persisted overrides so the flashed defaults stick across reboot.
    for (uint32_t p = 0; p < ZIP_KEYBIND_PARAM_COUNT; p++) {
        char key[24];
        zip_keybind_settings_key(key, sizeof(key), id, p);
        settings_delete(key);
    }

    // Restore default direction bindings and clear their NVS entries.
    for (uint32_t d = 0; d < cfg->num_bindings && d < ZIP_KEYBIND_DIR_COUNT; d++) {
        cfg->bindings[d] = cfg->default_bindings[d];
    }
    for (uint32_t d = 0; d < ZIP_KEYBIND_DIR_COUNT; d++) {
        char gkey[24];
        zip_keybind_gkb_key(gkey, sizeof(gkey), id, d);
        settings_delete(gkey);
    }
    return 0;
}

// settings_load() callback. name is the part after the subtree, "<id>/<param>".
static int zip_keybind_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    const char *slash = strchr(name, '/');
    if (!slash) {
        return -ENOENT;
    }
    uint32_t id = (uint32_t)strtoul(name, NULL, 10);
    uint32_t param = (uint32_t)strtoul(slash + 1, NULL, 10);

    int32_t value;
    if (len != sizeof(value)) {
        return -EINVAL;
    }
    ssize_t rc = read_cb(cb_arg, &value, sizeof(value));
    if (rc < 0) {
        return (int)rc;
    }
    // Re-apply into RAM (no re-save: this is the load path).
    zip_keybind_set_param(id, (enum zip_keybind_param)param, value);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(zip_keybind, ZIP_KEYBIND_SETTINGS_SUBTREE, NULL,
                               zip_keybind_settings_set, NULL, NULL);

#endif /* CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND_STUDIO_RPC */