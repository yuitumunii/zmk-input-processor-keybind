/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Dynamic (layer-aware) trackball gesture input processor.
 * compatible: "zmk,input-processor-keybind-dynamic"
 *
 * Each event inspects the currently active layer and dispatches to the
 * per-layer binding table (gkb_table[]).  If the active layer has no
 * enabled entry the event is passed through unchanged (CONTINUE) so the
 * normal cursor processing works as usual.  When a layer does have an
 * entry the direction is quantised and the matching binding is fired
 * (STOP — cursor movement is suppressed).
 *
 * Timing / accumulation logic is identical to input-processor-keybind.c.
 * The only difference is that cfg->bindings[] is replaced by
 * gkb_table[active_layer].bindings[].
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
#include <zephyr/settings/settings.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zmk/pointing/gesture_layer.h>

#define DT_DRV_COMPAT zmk_input_processor_keybind_dynamic

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* -----------------------------------------------------------------------
 * Global layer binding table (shared across all instances; there is only
 * one zip_keybind_dynamic node in the DTS).
 * ----------------------------------------------------------------------- */

#define GKB_DIR_COUNT   4
#define GKB_MAX_LAYERS  16

/* Per-direction tunable sensitivity (TICK/WAIT_MS/THRESHOLD). */
struct gkb_dir_sens {
    uint32_t tick;
    uint32_t wait_ms;
    int32_t  threshold;
    int64_t  last_fire;   /* runtime only: uptime ms of last fire (cooldown gate) */
};

struct gkb_layer_entry {
    bool enabled;
    bool sens_seeded;     /* per-dir sens initialised from defaults? */
    struct zmk_behavior_binding bindings[GKB_DIR_COUNT];
    struct gkb_dir_sens sens[GKB_DIR_COUNT];
};

static struct gkb_layer_entry gkb_table[GKB_MAX_LAYERS];

/* Default per-direction sensitivity, captured from the dynamic processor
 * devicetree config at init. Used to seed a layer's sens the first time it
 * is touched, and as the target for gkb_layer_reset_sens(). */
static struct gkb_dir_sens gkb_sens_defaults = {
    .tick = 180, .wait_ms = 0, .threshold = 1, .last_fire = 0,
};

/* Seed a layer's per-direction sensitivity from the defaults (idempotent). */
static void gkb_seed_layer_sens(uint8_t layer) {
    if (layer >= GKB_MAX_LAYERS || gkb_table[layer].sens_seeded) return;
    for (int d = 0; d < GKB_DIR_COUNT; d++) {
        gkb_table[layer].sens[d].tick      = gkb_sens_defaults.tick;
        gkb_table[layer].sens[d].wait_ms   = gkb_sens_defaults.wait_ms;
        gkb_table[layer].sens[d].threshold = gkb_sens_defaults.threshold;
        gkb_table[layer].sens[d].last_fire = 0;
    }
    gkb_table[layer].sens_seeded = true;
}

/* list of configured (ever set) layers for gkb_layer_at() */
static uint8_t gkb_configured_layers[GKB_MAX_LAYERS];
static int     gkb_configured_count = 0;

static void gkb_mark_configured(uint8_t layer) {
    gkb_seed_layer_sens(layer);
    for (int i = 0; i < gkb_configured_count; i++) {
        if (gkb_configured_layers[i] == layer) return;
    }
    if (gkb_configured_count < GKB_MAX_LAYERS) {
        gkb_configured_layers[gkb_configured_count++] = layer;
    }
}

/* -----------------------------------------------------------------------
 * Key-state enum  (identical to input-processor-keybind.c)
 * ----------------------------------------------------------------------- */
enum zip_keybind_key_state {
    ZIP_KEY_NONE       = 0,
    ZIP_KEY_RIGHT      = 1,
    ZIP_KEY_LEFT       = 2,
    ZIP_KEY_HORIZONTAL = BIT(ZIP_KEY_LEFT) | BIT(ZIP_KEY_RIGHT),
    ZIP_KEY_DOWN       = 3,
    ZIP_KEY_UP         = 4,
    ZIP_KEY_VERTICAL   = BIT(ZIP_KEY_UP) | BIT(ZIP_KEY_DOWN),
};

/* -----------------------------------------------------------------------
 * Config / data structs
 * ----------------------------------------------------------------------- */
struct gkb_dynamic_config {
    uint8_t  index;
    uint8_t  mode;           /* 0=raw, 1=4way, 2=8way */
    bool     track_remainders;
    uint32_t tap_ms;
    uint32_t wait_ms;
    uint32_t tick;
    int32_t  threshold;
    int32_t  max_threshold;
    int32_t  max_pending_activations;
    /* guard layers: events from these layers always pass through */
    const uint8_t *guard_layers;
    uint8_t         guard_layers_len;
};

struct gkb_dynamic_data {
    int32_t  delta_x;
    int32_t  delta_y;
    int32_t  last_delta_x;
    int32_t  last_delta_y;
    int32_t  max_delta;
    uint8_t  device_index;
    uint8_t  active_layer;     /* layer captured at handle_event time */
    enum zip_keybind_key_state state;

    /* runtime-tunable (seeded from config at init) */
    uint32_t tick;
    uint32_t wait_ms;
    uint32_t tap_ms;
    int32_t  threshold;
    int32_t  max_threshold;

    const struct device *dev;
    struct k_work_delayable press_work;
};

/* -----------------------------------------------------------------------
 * Motion helpers  (verbatim from input-processor-keybind.c)
 * ----------------------------------------------------------------------- */
/* dir index (matches bindings[]/sens[] order): 0=RIGHT 1=LEFT 2=DOWN 3=UP. */
static inline int gkb_dir_for_x(int32_t dx) { return dx > 0 ? 0 : 1; }
static inline int gkb_dir_for_y(int32_t dy) { return dy > 0 ? 3 : 2; }

/* Per-(layer,dir) tick (>=1). Falls back to global data->tick if unseeded. */
static inline uint32_t gkb_tick_at(const struct gkb_dynamic_data *data, int dir) {
    uint8_t L = data->active_layer;
    if (L >= GKB_MAX_LAYERS || !gkb_table[L].sens_seeded) {
        return data->tick ? data->tick : 1;
    }
    uint32_t t = gkb_table[L].sens[dir].tick;
    return t ? t : 1;
}

static inline bool gkb_has_pending(const struct gkb_dynamic_data *data) {
    uint32_t tx = gkb_tick_at(data, gkb_dir_for_x(data->delta_x));
    uint32_t ty = gkb_tick_at(data, gkb_dir_for_y(data->delta_y));
    return abs(data->delta_x) >= (int32_t)tx || abs(data->delta_y) >= (int32_t)ty;
}

static uint32_t gkb_approx_hypot(uint32_t a, uint32_t b) {
    if (a < b) { uint32_t t = a; a = b; b = t; }
    return a + ((b * 3) >> 3);
}

static void gkb_handle_4way(struct gkb_dynamic_data *data) {
    int32_t movement = (int32_t)gkb_approx_hypot((uint32_t)abs(data->last_delta_x),
                                                   (uint32_t)abs(data->last_delta_y));
    if (abs(data->last_delta_x) > abs(data->last_delta_y)) {
        if (data->last_delta_x < 0) movement = -movement;
        data->delta_x = CLAMP(data->delta_x + movement, -data->max_delta, data->max_delta);
    } else {
        if (data->last_delta_y < 0) movement = -movement;
        data->delta_y = CLAMP(data->delta_y + movement, -data->max_delta, data->max_delta);
    }
    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

static void gkb_handle_raw(struct gkb_dynamic_data *data) {
    data->delta_x = CLAMP(data->delta_x + data->last_delta_x, -data->max_delta, data->max_delta);
    data->delta_y = CLAMP(data->delta_y + data->last_delta_y, -data->max_delta, data->max_delta);
    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

static void gkb_handle_8way(struct gkb_dynamic_data *data) {
    int32_t dx  = data->last_delta_x;
    int32_t dy  = data->last_delta_y;
    int32_t adx = abs(dx);
    int32_t ady = abs(dy);
    if (dy == 0) {
        data->delta_x = CLAMP(data->delta_x + dx, -data->max_delta, data->max_delta);
    } else if (dx == 0) {
        data->delta_y = CLAMP(data->delta_y + dy, -data->max_delta, data->max_delta);
    } else {
        int32_t movement = (int32_t)gkb_approx_hypot((uint32_t)adx, (uint32_t)ady);
        if (5 * adx > 12 * ady) {
            data->delta_x = CLAMP(data->delta_x + (dx < 0 ? -movement : movement),
                                  -data->max_delta, data->max_delta);
        } else if (5 * ady > 12 * adx) {
            data->delta_y = CLAMP(data->delta_y + (dy < 0 ? -movement : movement),
                                  -data->max_delta, data->max_delta);
        } else {
            data->delta_x = CLAMP(data->delta_x + (dx < 0 ? -movement : movement),
                                  -data->max_delta, data->max_delta);
            data->delta_y = CLAMP(data->delta_y + (dy < 0 ? -movement : movement),
                                  -data->max_delta, data->max_delta);
        }
    }
    data->last_delta_x = 0;
    data->last_delta_y = 0;
}

/* -----------------------------------------------------------------------
 * Binding invocation  (uses gkb_table rather than cfg->bindings)
 * ----------------------------------------------------------------------- */
static inline uint32_t gkb_get_position(const struct gkb_dynamic_data *data,
                                         const struct gkb_dynamic_config *cfg) {
    return ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(data->device_index, cfg->index);
}

static int gkb_invoke_binding(struct gkb_dynamic_data *data,
                              const struct gkb_dynamic_config *cfg,
                              int idx, bool pressed) {
    if ((pressed  && (data->state & BIT(idx))) ||
        (!pressed && !(data->state & BIT(idx))))
        return 0;

    int bindingId = idx - 1; /* normalise: RIGHT=1 → index 0 */
    const struct zmk_behavior_binding *b =
        &gkb_table[data->active_layer].bindings[bindingId];

    struct zmk_behavior_binding_event bev = {
        .position  = gkb_get_position(data, cfg),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_invoke_binding(b, bev, pressed);
    if (ret == 0) {
        if (pressed) data->state |= BIT(idx);
        else         data->state &= ~BIT(idx);
    }

    LOG_DBG("dynamic binding: bh: %s 0x%02x 0x%02x pressed: %s state %X",
            b->behavior_dev, b->param1, b->param2,
            pressed ? "true" : "false", data->state);
    return ret;
}

static inline void gkb_check_release(struct gkb_dynamic_data *data,
                                      const struct gkb_dynamic_config *cfg, int idx) {
    if (data->state & BIT(idx))
        gkb_invoke_binding(data, cfg, idx, false);
}

/* -----------------------------------------------------------------------
 * press_work callback  (same logic as zip_keybind, but reads gkb_table)
 * ----------------------------------------------------------------------- */
static void gkb_press_work_cb(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct gkb_dynamic_data *data =
        CONTAINER_OF(d_work, struct gkb_dynamic_data, press_work);
    const struct device *dev = data->dev;
    const struct gkb_dynamic_config *cfg = dev->config;

    bool has_pending = gkb_has_pending(data);
    uint8_t L = data->active_layer;
    bool per_layer = (L < GKB_MAX_LAYERS && gkb_table[L].sens_seeded);
    int64_t now = k_uptime_get();

    /* Release the previous cycle's presses. Cooldown is now per-direction
     * (timestamp gate below) rather than a single blocking sleep, so a fast
     * direction stays fast while another can have a long cooldown. */
    if (data->state != ZIP_KEY_NONE) {
        gkb_check_release(data, cfg, ZIP_KEY_LEFT);
        gkb_check_release(data, cfg, ZIP_KEY_RIGHT);
        gkb_check_release(data, cfg, ZIP_KEY_UP);
        gkb_check_release(data, cfg, ZIP_KEY_DOWN);
        if (!per_layer) {
            k_sleep(K_MSEC(data->wait_ms)); /* legacy global cooldown */
        }
    }

    if (has_pending) {
        int idx = ZIP_KEY_NONE;
        int idy = ZIP_KEY_NONE;

        /* ---- horizontal axis ---- */
        int dx_dir = gkb_dir_for_x(data->delta_x);   /* 0=RIGHT 1=LEFT */
        uint32_t tick_x = gkb_tick_at(data, dx_dir);
        if (abs(data->delta_x) >= (int32_t)tick_x) {
            uint32_t wait_x = per_layer ? gkb_table[L].sens[dx_dir].wait_ms : 0;
            bool cooled = !per_layer ||
                          (now - gkb_table[L].sens[dx_dir].last_fire >= (int64_t)wait_x);
            if (data->delta_x > 0) {
                data->delta_x -= tick_x;
                gkb_check_release(data, cfg, ZIP_KEY_LEFT);
                if (cooled) idx = ZIP_KEY_RIGHT;
            } else {
                data->delta_x += tick_x;
                gkb_check_release(data, cfg, ZIP_KEY_RIGHT);
                if (cooled) idx = ZIP_KEY_LEFT;
            }
            if (cooled && per_layer) gkb_table[L].sens[dx_dir].last_fire = now;
        }

        /* ---- vertical axis ---- */
        int dy_dir = gkb_dir_for_y(data->delta_y);   /* 3=UP 2=DOWN */
        uint32_t tick_y = gkb_tick_at(data, dy_dir);
        if (abs(data->delta_y) >= (int32_t)tick_y) {
            uint32_t wait_y = per_layer ? gkb_table[L].sens[dy_dir].wait_ms : 0;
            bool cooled = !per_layer ||
                          (now - gkb_table[L].sens[dy_dir].last_fire >= (int64_t)wait_y);
            if (data->delta_y > 0) {
                data->delta_y -= tick_y;
                gkb_check_release(data, cfg, ZIP_KEY_DOWN);
                if (cooled) idy = ZIP_KEY_UP;
            } else {
                data->delta_y += tick_y;
                gkb_check_release(data, cfg, ZIP_KEY_UP);
                if (cooled) idy = ZIP_KEY_DOWN;
            }
            if (cooled && per_layer) gkb_table[L].sens[dy_dir].last_fire = now;
        }

        if (idx != ZIP_KEY_NONE) gkb_invoke_binding(data, cfg, idx, true);
        if (idy != ZIP_KEY_NONE) gkb_invoke_binding(data, cfg, idy, true);

        k_work_schedule(&data->press_work, K_MSEC(data->tap_ms));
    }

    if (!cfg->track_remainders) {
        data->delta_x = 0;
        data->delta_y = 0;
    }
}

/* -----------------------------------------------------------------------
 * handle_event
 * ----------------------------------------------------------------------- */
static int gkb_handle_event(const struct device *dev, struct input_event *event,
                             uint32_t param1, uint32_t param2,
                             struct zmk_input_processor_state *state) {
    const struct gkb_dynamic_config *cfg = dev->config;
    struct gkb_dynamic_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* global upper bound: ignore implausibly large single jumps */
    if (abs(event->value) > data->max_threshold) {
        return ZMK_INPUT_PROC_STOP;
    }

    /* identify active layer */
    zmk_keymap_layer_id_t lid =
        zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active());

    /* guard-layer check: always pass through on guard layers */
    for (uint8_t i = 0; i < cfg->guard_layers_len; i++) {
        if (lid == cfg->guard_layers[i]) {
            return ZMK_INPUT_PROC_CONTINUE;
        }
    }

    /* no entry → pass through (cursor normal) */
    if (lid >= GKB_MAX_LAYERS || !gkb_table[lid].enabled) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* per-direction lower threshold (noise gate). direction from event sign. */
    int ev_dir;
    if (event->code == INPUT_REL_X) {
        ev_dir = gkb_dir_for_x(event->value);
    } else if (event->code == INPUT_REL_Y) {
        ev_dir = gkb_dir_for_y(event->value);
    } else {
        return ZMK_INPUT_PROC_CONTINUE;
    }
    int32_t th = gkb_table[lid].sens_seeded ? gkb_table[lid].sens[ev_dir].threshold
                                            : data->threshold;
    if (th > abs(event->value)) {
        return ZMK_INPUT_PROC_STOP;
    }

    /* accumulate movement */
    if (event->code == INPUT_REL_X) {
        data->last_delta_x = event->value;
    } else {
        data->last_delta_y = event->value;
    }

    /* wait until full movement received */
    if (!event->sync) {
        return ZMK_INPUT_PROC_STOP;
    }

    /* capture active layer for press_work_cb */
    data->active_layer = (uint8_t)lid;

    /* size the accumulation clamp for the largest per-direction tick so no
     * direction is starved of headroom. */
    if (gkb_table[lid].sens_seeded) {
        uint32_t maxt = 1;
        for (int d = 0; d < GKB_DIR_COUNT; d++) {
            if (gkb_table[lid].sens[d].tick > maxt) maxt = gkb_table[lid].sens[d].tick;
        }
        data->max_delta = cfg->max_pending_activations * (int32_t)maxt;
    }

    if (cfg->mode == 0) {
        gkb_handle_raw(data);
    } else if (cfg->mode == 1) {
        gkb_handle_4way(data);
    } else {
        gkb_handle_8way(data);
    }

    LOG_DBG("dynamic: lid=%u dx=%d dy=%d tick=%d",
            lid, data->delta_x, data->delta_y, data->tick);

    if (gkb_has_pending(data)) {
        data->device_index = state->input_device_index;
        if (!k_work_delayable_is_pending(&data->press_work)) {
            k_work_schedule(&data->press_work, K_NO_WAIT);
        }
    }

    return ZMK_INPUT_PROC_STOP;
}

static struct zmk_input_processor_driver_api gkb_dynamic_driver_api = {
    .handle_event = gkb_handle_event,
};

/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */
static int gkb_dynamic_init(const struct device *dev) {
    struct gkb_dynamic_data *data = dev->data;
    const struct gkb_dynamic_config *cfg = dev->config;

    data->dev       = dev;
    data->tick      = cfg->tick;
    data->wait_ms   = cfg->wait_ms;
    data->tap_ms    = cfg->tap_ms;
    data->threshold = cfg->threshold;
    data->max_threshold = cfg->max_threshold;
    data->max_delta = cfg->max_pending_activations * data->tick;

    /* Capture per-direction sensitivity defaults from devicetree config so
     * gkb_seed_layer_sens()/reset use the real flashed values. */
    gkb_sens_defaults.tick      = cfg->tick ? cfg->tick : 1;
    gkb_sens_defaults.wait_ms   = cfg->wait_ms;
    gkb_sens_defaults.threshold = cfg->threshold;
    gkb_sens_defaults.last_fire = 0;

    /* Seed any layers already marked configured (e.g. NVS bindings loaded
     * before this init runs would have seeded with the static defaults; this
     * is harmless because seed is idempotent and gls NVS overrides later). */
    for (uint8_t l = 0; l < GKB_MAX_LAYERS; l++) {
        if (gkb_table[l].sens_seeded) {
            for (int d = 0; d < GKB_DIR_COUNT; d++) {
                /* keep any value already overridden from NVS; only fill zeros */
                if (gkb_table[l].sens[d].tick == 0) {
                    gkb_table[l].sens[d].tick = gkb_sens_defaults.tick;
                }
            }
        }
    }

    k_work_init_delayable(&data->press_work, gkb_press_work_cb);
    return 0;
}

/* -----------------------------------------------------------------------
 * Device-tree instantiation macro
 * ----------------------------------------------------------------------- */
#define GKB_GUARD_LAYERS(n)    DT_INST_PROP_OR(n, guard_layers, {})
#define GKB_GUARD_LAYERS_LEN(n) DT_INST_PROP_LEN_OR(n, guard_layers, 0)

#define GKB_DYNAMIC_INST(n)                                                                    \
    static struct gkb_dynamic_data gkb_dynamic_data_##n = {                                    \
        .delta_x = 0, .delta_y = 0, .state = ZIP_KEY_NONE,                                    \
    };                                                                                         \
    static const uint8_t gkb_guard_layers_##n[] = DT_INST_PROP_OR(n, guard_layers, {});       \
    static const struct gkb_dynamic_config gkb_dynamic_config_##n = {                          \
        .index               = n,                                                              \
        .mode                = DT_INST_PROP_OR(n, mode, 1),                                   \
        .track_remainders    = DT_INST_PROP_OR(n, track_remainders, false),                   \
        .tap_ms              = DT_INST_PROP_OR(n, tap_ms, 20),                                \
        .wait_ms             = DT_INST_PROP_OR(n, wait_ms, 0),                                \
        .tick                = DT_INST_PROP_OR(n, tick, 180),                                 \
        .threshold           = DT_INST_PROP_OR(n, threshold, 1),                              \
        .max_threshold       = DT_INST_PROP_OR(n, max_threshold, 200),                        \
        .max_pending_activations = DT_INST_PROP_OR(n, max_pending_activations, 5),            \
        .guard_layers        = gkb_guard_layers_##n,                                          \
        .guard_layers_len    = ARRAY_SIZE(gkb_guard_layers_##n),                              \
    };                                                                                         \
    DEVICE_DT_INST_DEFINE(n, &gkb_dynamic_init, NULL,                                         \
                          &gkb_dynamic_data_##n, &gkb_dynamic_config_##n,                     \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                   \
                          &gkb_dynamic_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GKB_DYNAMIC_INST)

/* -----------------------------------------------------------------------
 * Runtime API  (declared in include/zmk/pointing/gesture_layer.h)
 * ----------------------------------------------------------------------- */

int gkb_layer_get_binding(uint8_t layer, uint8_t dir,
                          uint16_t *bid, int32_t *p1, int32_t *p2) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    const struct zmk_behavior_binding *b = &gkb_table[layer].bindings[dir];
    if (bid) *bid = b->behavior_dev ? zmk_behavior_get_local_id(b->behavior_dev) : 0;
    if (p1)  *p1  = (int32_t)b->param1;
    if (p2)  *p2  = (int32_t)b->param2;
    return 0;
}

int gkb_layer_set_binding(uint8_t layer, uint8_t dir,
                          uint16_t bid, int32_t p1, int32_t p2) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    const char *name = zmk_behavior_find_behavior_name_from_local_id(bid);
    if (!name) return -ENODEV;
    const struct device *bdev = zmk_behavior_get_binding(name);
    if (!bdev) return -ENODEV;
    gkb_table[layer].bindings[dir].behavior_dev = bdev->name;
    gkb_table[layer].bindings[dir].param1 = (uint32_t)p1;
    gkb_table[layer].bindings[dir].param2 = (uint32_t)p2;
    gkb_mark_configured(layer);
    return 0;
}

int gkb_layer_enable(uint8_t layer) {
    if (layer >= GKB_MAX_LAYERS) return -EINVAL;
    gkb_table[layer].enabled = true;
    gkb_mark_configured(layer);
    return 0;
}

int gkb_layer_disable(uint8_t layer) {
    if (layer >= GKB_MAX_LAYERS) return -EINVAL;
    gkb_table[layer].enabled = false;
    return 0;
}

int gkb_layer_count(void) { return gkb_configured_count; }

int gkb_layer_at(int index, uint8_t *layer_out, bool *enabled_out) {
    if (index < 0 || index >= gkb_configured_count) return -EINVAL;
    if (layer_out)   *layer_out   = gkb_configured_layers[index];
    if (enabled_out) *enabled_out = gkb_table[gkb_configured_layers[index]].enabled;
    return 0;
}

/* -----------------------------------------------------------------------
 * NVS persistence  (subtree "glk")
 * Keys:
 *   glk/<layer>/<dir>  → struct gkb_binding_nvs  (name + params)
 *   glk/<layer>/e      → int32_t enabled (0 or 1)
 * ----------------------------------------------------------------------- */

#define GKB_NVS_SUBTREE    "glk"
#define GKB_NVS_NAME_MAX   24
#define GKB_NVS_ENABLED_DIR 0xFF  /* magic dir value for the enable flag */

struct gkb_binding_nvs {
    char    name[GKB_NVS_NAME_MAX];
    int32_t param1;
    int32_t param2;
};

static void gkb_nvs_binding_key(char *buf, size_t len, uint8_t layer, uint8_t dir) {
    snprintf(buf, len, GKB_NVS_SUBTREE "/%u/%u", (unsigned)layer, (unsigned)dir);
}

static void gkb_nvs_enable_key(char *buf, size_t len, uint8_t layer) {
    snprintf(buf, len, GKB_NVS_SUBTREE "/%u/e", (unsigned)layer);
}

int gkb_layer_save_binding(uint8_t layer, uint8_t dir) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    const struct zmk_behavior_binding *b = &gkb_table[layer].bindings[dir];
    struct gkb_binding_nvs rec = {0};
    if (b->behavior_dev)
        strncpy(rec.name, b->behavior_dev, sizeof(rec.name) - 1);
    rec.param1 = (int32_t)b->param1;
    rec.param2 = (int32_t)b->param2;
    char key[32];
    gkb_nvs_binding_key(key, sizeof(key), layer, dir);
    return settings_save_one(key, &rec, sizeof(rec));
}

int gkb_layer_save_enable(uint8_t layer) {
    if (layer >= GKB_MAX_LAYERS) return -EINVAL;
    int32_t val = gkb_table[layer].enabled ? 1 : 0;
    char key[32];
    gkb_nvs_enable_key(key, sizeof(key), layer);
    return settings_save_one(key, &val, sizeof(val));
}

/* Settings load callback.  name is subtree-relative: "<layer>/<dir>" or "<layer>/e" */
static int gkb_nvs_set(const char *name, size_t len,
                        settings_read_cb read_cb, void *cb_arg) {
    const char *slash = strchr(name, '/');
    if (!slash) return -ENOENT;

    uint8_t layer = (uint8_t)strtoul(name, NULL, 10);
    if (layer >= GKB_MAX_LAYERS) return 0; /* out of range, ignore safely */

    /* enabled flag: "<layer>/e" */
    if (*(slash + 1) == 'e' && *(slash + 2) == '\0') {
        int32_t val = 0;
        if (len != sizeof(val)) return -EINVAL;
        ssize_t rc = read_cb(cb_arg, &val, sizeof(val));
        if (rc < 0) return (int)rc;
        gkb_table[layer].enabled = (val != 0);
        gkb_mark_configured(layer);
        return 0;
    }

    /* binding: "<layer>/<dir>" */
    uint8_t dir = (uint8_t)strtoul(slash + 1, NULL, 10);
    if (dir >= GKB_DIR_COUNT) return 0;

    struct gkb_binding_nvs rec;
    if (len != sizeof(rec)) return -EINVAL;
    ssize_t rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) return (int)rc;
    rec.name[sizeof(rec.name) - 1] = '\0';

    const struct device *bdev = zmk_behavior_get_binding(rec.name);
    if (!bdev) return 0; /* behavior not found – degrade gracefully */

    gkb_table[layer].bindings[dir].behavior_dev = bdev->name;
    gkb_table[layer].bindings[dir].param1 = (uint32_t)rec.param1;
    gkb_table[layer].bindings[dir].param2 = (uint32_t)rec.param2;
    gkb_mark_configured(layer);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gkb_dynamic, GKB_NVS_SUBTREE, NULL,
                                gkb_nvs_set, NULL, NULL);

/* -----------------------------------------------------------------------
 * Sensitivity API  (declared in include/zmk/pointing/gesture_layer.h)
 * Operates on all dynamic processor instances (there is normally only one).
 * ----------------------------------------------------------------------- */

#define GKB_DYNAMIC_DEV(n) DEVICE_DT_INST_GET(n),
static const struct device *const gkb_dynamic_devs[] = {
    DT_INST_FOREACH_STATUS_OKAY(GKB_DYNAMIC_DEV)
};
#define GKB_DYNAMIC_DEV_COUNT ARRAY_SIZE(gkb_dynamic_devs)

#define GKS_NVS_SUBTREE "gks"

int gkb_get_sensitivity(struct gkb_sensitivity *out) {
    if (!out || GKB_DYNAMIC_DEV_COUNT == 0) {
        return -EINVAL;
    }
    const struct gkb_dynamic_data *d = gkb_dynamic_devs[0]->data;
    out->tick          = d->tick;
    out->wait_ms       = d->wait_ms;
    out->tap_ms        = d->tap_ms;
    out->threshold     = d->threshold;
    out->max_threshold = d->max_threshold;
    return 0;
}

static int gkb_apply_param(struct gkb_dynamic_data *data,
                           const struct gkb_dynamic_config *cfg,
                           enum gkb_param param, int32_t value) {
    switch (param) {
    case GKB_PARAM_TICK:
        data->tick      = (uint32_t)MAX(value, 1);
        data->max_delta = cfg->max_pending_activations * data->tick;
        break;
    case GKB_PARAM_WAIT_MS:
        data->wait_ms   = (uint32_t)MAX(value, 0);
        break;
    case GKB_PARAM_TAP_MS:
        data->tap_ms    = (uint32_t)MAX(value, 0);
        break;
    case GKB_PARAM_THRESHOLD:
        data->threshold = value;
        break;
    case GKB_PARAM_MAX_THRESHOLD:
        data->max_threshold = value;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

int gkb_set_param(enum gkb_param param, int32_t value) {
    if (GKB_DYNAMIC_DEV_COUNT == 0) {
        return -EINVAL;
    }
    for (size_t i = 0; i < GKB_DYNAMIC_DEV_COUNT; i++) {
        const struct device *dev = gkb_dynamic_devs[i];
        struct gkb_dynamic_data *data = dev->data;
        const struct gkb_dynamic_config *cfg = dev->config;
        int ret = gkb_apply_param(data, cfg, param, value);
        if (ret < 0) {
            return ret;
        }
    }
    return 0;
}

int gkb_save_param(enum gkb_param param, int32_t value) {
    char key[24];
    snprintf(key, sizeof(key), GKS_NVS_SUBTREE "/%u", (unsigned)param);
    return settings_save_one(key, &value, sizeof(value));
}

int gkb_reset_sensitivity(void) {
    if (GKB_DYNAMIC_DEV_COUNT == 0) {
        return -EINVAL;
    }
    for (size_t i = 0; i < GKB_DYNAMIC_DEV_COUNT; i++) {
        const struct device *dev = gkb_dynamic_devs[i];
        const struct gkb_dynamic_config *cfg = dev->config;
        struct gkb_dynamic_data *data = dev->data;
        data->tick          = cfg->tick;
        data->wait_ms       = cfg->wait_ms;
        data->tap_ms        = cfg->tap_ms;
        data->threshold     = cfg->threshold;
        data->max_threshold = cfg->max_threshold;
        data->max_delta     = cfg->max_pending_activations * data->tick;
    }
    for (uint32_t p = 0; p < 5; p++) {
        char key[24];
        snprintf(key, sizeof(key), GKS_NVS_SUBTREE "/%u", (unsigned)p);
        settings_delete(key);
    }
    return 0;
}

static int gks_nvs_set(const char *name, size_t len,
                       settings_read_cb read_cb, void *cb_arg) {
    uint32_t param = (uint32_t)strtoul(name, NULL, 10);
    int32_t value;
    if (len != sizeof(value)) {
        return -EINVAL;
    }
    ssize_t rc = read_cb(cb_arg, &value, sizeof(value));
    if (rc < 0) {
        return (int)rc;
    }
    gkb_set_param((enum gkb_param)param, value);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gkb_sens, GKS_NVS_SUBTREE, NULL,
                                gks_nvs_set, NULL, NULL);

/* -----------------------------------------------------------------------
 * Per-(layer, direction) sensitivity  (subtree "gls")
 *   gls/<layer>/<dir>  → struct gkb_sens_nvs {tick, wait_ms, threshold}
 * Only TICK/WAIT_MS/THRESHOLD are per-direction; TAP_MS/MAX_THRESHOLD global.
 * ----------------------------------------------------------------------- */

#define GLS_NVS_SUBTREE "gls"

struct gkb_sens_nvs {
    uint32_t tick;
    uint32_t wait_ms;
    int32_t  threshold;
};

int gkb_layer_get_sens(uint8_t layer, uint8_t dir, struct gkb_dir_sensitivity *out) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT || !out) return -EINVAL;
    gkb_seed_layer_sens(layer);
    out->tick      = gkb_table[layer].sens[dir].tick;
    out->wait_ms   = gkb_table[layer].sens[dir].wait_ms;
    out->threshold = gkb_table[layer].sens[dir].threshold;
    return 0;
}

int gkb_layer_set_sens(uint8_t layer, uint8_t dir, enum gkb_param param, int32_t value) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    gkb_seed_layer_sens(layer);
    struct gkb_dir_sens *s = &gkb_table[layer].sens[dir];
    switch (param) {
    case GKB_PARAM_TICK:      s->tick      = (uint32_t)MAX(value, 1); break;
    case GKB_PARAM_WAIT_MS:   s->wait_ms   = (uint32_t)MAX(value, 0); break;
    case GKB_PARAM_THRESHOLD: s->threshold = value;                   break;
    default: return -EINVAL; /* TAP_MS / MAX_THRESHOLD are global only */
    }
    gkb_mark_configured(layer);
    return 0;
}

static void gls_nvs_key(char *buf, size_t len, uint8_t layer, uint8_t dir) {
    snprintf(buf, len, GLS_NVS_SUBTREE "/%u/%u", (unsigned)layer, (unsigned)dir);
}

int gkb_layer_save_sens(uint8_t layer, uint8_t dir) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    gkb_seed_layer_sens(layer);
    struct gkb_sens_nvs rec = {
        .tick      = gkb_table[layer].sens[dir].tick,
        .wait_ms   = gkb_table[layer].sens[dir].wait_ms,
        .threshold = gkb_table[layer].sens[dir].threshold,
    };
    char key[32];
    gls_nvs_key(key, sizeof(key), layer, dir);
    return settings_save_one(key, &rec, sizeof(rec));
}

int gkb_layer_reset_sens(uint8_t layer, uint8_t dir) {
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return -EINVAL;
    gkb_seed_layer_sens(layer);
    gkb_table[layer].sens[dir].tick      = gkb_sens_defaults.tick;
    gkb_table[layer].sens[dir].wait_ms   = gkb_sens_defaults.wait_ms;
    gkb_table[layer].sens[dir].threshold = gkb_sens_defaults.threshold;
    char key[32];
    gls_nvs_key(key, sizeof(key), layer, dir);
    settings_delete(key);
    return 0;
}

static int gls_nvs_set(const char *name, size_t len,
                       settings_read_cb read_cb, void *cb_arg) {
    const char *slash = strchr(name, '/');
    if (!slash) return -ENOENT;
    uint8_t layer = (uint8_t)strtoul(name, NULL, 10);
    uint8_t dir   = (uint8_t)strtoul(slash + 1, NULL, 10);
    if (layer >= GKB_MAX_LAYERS || dir >= GKB_DIR_COUNT) return 0;

    struct gkb_sens_nvs rec;
    if (len != sizeof(rec)) return -EINVAL;
    ssize_t rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) return (int)rc;

    gkb_seed_layer_sens(layer);
    gkb_table[layer].sens[dir].tick      = rec.tick ? rec.tick : 1;
    gkb_table[layer].sens[dir].wait_ms   = rec.wait_ms;
    gkb_table[layer].sens[dir].threshold = rec.threshold;
    gkb_mark_configured(layer);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gkb_layer_sens, GLS_NVS_SUBTREE, NULL,
                                gls_nvs_set, NULL, NULL);
