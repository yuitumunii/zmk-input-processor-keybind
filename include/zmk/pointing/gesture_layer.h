/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Runtime API for the dynamic (layer-aware) trackball gesture processor.
 * Implemented in src/pointing/input-processor-keybind-dynamic.c.
 *
 * The global table gkb_table[GKB_MAX_LAYERS] holds per-layer bindings.
 * All changes take effect immediately; call gkb_layer_save_* to persist
 * across reboot via Zephyr settings (NVS).
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Number of directions per layer entry (RIGHT, LEFT, DOWN, UP). */
#define GKB_DIR_COUNT  4

/* Maximum number of distinct layers that can have entries. */
#define GKB_MAX_LAYERS 16

/**
 * @brief Read the current binding for a layer/direction.
 *
 * @param layer   Layer index (0-based, < GKB_MAX_LAYERS).
 * @param dir     Direction index: 0=RIGHT 1=LEFT 2=DOWN 3=UP.
 * @param bid     Output: behavior local-id (same crc16 as ZMK Studio uses).
 * @param p1      Output: param1.
 * @param p2      Output: param2.
 * @return 0 on success, -EINVAL if layer/dir out of range.
 */
int gkb_layer_get_binding(uint8_t layer, uint8_t dir,
                          uint16_t *bid, int32_t *p1, int32_t *p2);

/**
 * @brief Live-replace the binding for a layer/direction.
 *
 * @param layer       Layer index.
 * @param dir         Direction index.
 * @param bid         Behavior local-id.
 * @param p1          param1.
 * @param p2          param2.
 * @return 0 on success, -EINVAL (range), -ENODEV (bid unresolvable).
 */
int gkb_layer_set_binding(uint8_t layer, uint8_t dir,
                          uint16_t bid, int32_t p1, int32_t p2);

/** Enable gesture processing for the given layer. */
int gkb_layer_enable(uint8_t layer);

/** Disable gesture processing for the given layer (cursor passes through). */
int gkb_layer_disable(uint8_t layer);

/** Save current binding for layer/dir to NVS. */
int gkb_layer_save_binding(uint8_t layer, uint8_t dir);

/** Save enabled flag for a layer to NVS. */
int gkb_layer_save_enable(uint8_t layer);

/**
 * @brief Number of layers that have ever been configured (set_binding or
 *        enable called, or loaded from NVS).
 */
int gkb_layer_count(void);

/**
 * @brief Return the layer index and enabled flag at position @p index in the
 *        configured-layer list.
 *
 * @param index       Position (0 .. gkb_layer_count()-1).
 * @param layer_out   Output layer index.
 * @param enabled_out Output enabled flag.
 * @return 0 on success, -EINVAL if index out of range.
 */
int gkb_layer_at(int index, uint8_t *layer_out, bool *enabled_out);

/* -----------------------------------------------------------------------
 * Sensitivity (tick / wait-ms / tap-ms / threshold / max-threshold) API
 * Implemented in src/pointing/input-processor-keybind-dynamic.c.
 * Values must match enum Param in proto/pyuron/gesture/gesture.proto.
 * ----------------------------------------------------------------------- */

enum gkb_param {
    GKB_PARAM_TICK          = 0,
    GKB_PARAM_WAIT_MS       = 1,
    GKB_PARAM_TAP_MS        = 2,
    GKB_PARAM_THRESHOLD     = 3,
    GKB_PARAM_MAX_THRESHOLD = 4,
};

struct gkb_sensitivity {
    uint32_t tick;
    uint32_t wait_ms;
    uint32_t tap_ms;
    int32_t  threshold;
    int32_t  max_threshold;
};

/** Read the current live sensitivity values from the dynamic processor. */
int gkb_get_sensitivity(struct gkb_sensitivity *out);

/** Live-change one sensitivity parameter. Takes effect immediately. */
int gkb_set_param(enum gkb_param param, int32_t value);

/** Persist one sensitivity parameter to NVS (survives reboot). */
int gkb_save_param(enum gkb_param param, int32_t value);

/** Restore all sensitivity values to devicetree defaults and clear NVS. */
int gkb_reset_sensitivity(void);
