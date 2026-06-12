/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Runtime registry/API for zip_keybind processor instances. Lets the custom
 * Studio RPC handler (src/studio/gesture_handler.c) read and live-change each
 * processor's tunable params without reflashing. Implemented in
 * src/pointing/input-processor-keybind.c.
 */

#pragma once

#include <stdint.h>

// Tunable parameters. Values MUST match enum Param in proto/pyuron/gesture/gesture.proto.
enum zip_keybind_param {
    ZIP_KEYBIND_PARAM_TICK = 0,
    ZIP_KEYBIND_PARAM_WAIT_MS = 1,
    ZIP_KEYBIND_PARAM_TAP_MS = 2,
    ZIP_KEYBIND_PARAM_THRESHOLD = 3,
    ZIP_KEYBIND_PARAM_MAX_THRESHOLD = 4,
};

struct zip_keybind_info {
    const char *name; // devicetree node name (static storage)
    uint32_t tick;
    uint32_t wait_ms;
    uint32_t tap_ms;
    int32_t threshold;
    int32_t max_threshold;
};

// Number of zip_keybind instances present on this build.
int zip_keybind_get_count(void);

// Fill *out with the current (live) state of instance `id`.
// Returns 0 on success, -EINVAL if id is out of range.
int zip_keybind_get_info(uint32_t id, struct zip_keybind_info *out);

// Live-change one parameter of instance `id`. Takes effect immediately.
// Returns 0 on success, -EINVAL on bad id/param.
int zip_keybind_set_param(uint32_t id, enum zip_keybind_param param, int32_t value);

// Persist one parameter to NVS so it survives reboot. Returns 0 on success.
int zip_keybind_save_param(uint32_t id, enum zip_keybind_param param, int32_t value);

// Restore instance `id` to its devicetree (flashed) defaults.
// Returns 0 on success, -EINVAL if id is out of range.
int zip_keybind_reset(uint32_t id);

#define ZIP_KEYBIND_DIR_COUNT 4 // right, left, down, up

// dir(0..3) の現在のbindingを読む。behavior_id は現在bindされている挙動の local id。
// 0成功 / -EINVAL(id,dir不正)。
int zip_keybind_get_binding(uint32_t id, uint32_t dir,
                            uint16_t *behavior_id, int32_t *param1, int32_t *param2);

// dir(0..3) を local id=behavior_id の挙動へライブ差し替え。
// 0成功 / -EINVAL(id,dir) / -ENODEV(behavior_id が解決不能)。
int zip_keybind_set_binding(uint32_t id, uint32_t dir,
                            uint16_t behavior_id, int32_t param1, int32_t param2);

// 現在の bindings[dir]（名前+param）を NVS "gkb/<id>/<dir>" に保存。0成功。
int zip_keybind_save_binding(uint32_t id, uint32_t dir);
