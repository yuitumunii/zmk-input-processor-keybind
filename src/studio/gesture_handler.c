/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Custom Studio RPC subsystem for live tuning of zip_keybind trackball
 * gestures (tick / wait-ms / threshold ...). Lets the ZMK Studio app change
 * gesture sensitivity at runtime, without reflashing. Built on the custom
 * subsystem support in the (cormoran) custom-studio-protocol ZMK base.
 *
 * Each response carries at most one GestureInfo so it fits the Studio RPC TX
 * buffer; the app probes get_count then get(0..count-1) instead of receiving
 * a list, so we need no async notifications here.
 */

#include <string.h>
#include <errno.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/sys/util.h>
#include <zmk/studio/custom.h>
#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND)
#include <zmk/pointing/zip_keybind.h>
#endif
#include <zmk/pointing/gesture_layer.h>
#include <pyuron/gesture/gesture.pb.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct zmk_rpc_custom_subsystem_meta gesture_feature_meta = {
    ZMK_RPC_CUSTOM_SUBSYSTEM_UI_URLS("https://github.com/yuitumunii/zmk-input-processor-keybind"),
    // Unsecured so the desktop app can tune without unlocking; these are
    // comfort params, not security-sensitive.
    .security = ZMK_STUDIO_RPC_HANDLER_UNSECURED,
};

ZMK_RPC_CUSTOM_SUBSYSTEM(pyuron_gesture, &gesture_feature_meta, gesture_rpc_handle_request);

ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER(pyuron_gesture, pyuron_gesture_Response);

// Fill a protobuf GestureInfo from the live state of the dynamic processor.
// id is accepted for API compatibility but ignored (there is one global set).
static int fill_gesture_info(uint32_t id, pyuron_gesture_GestureInfo *info) {
    struct gkb_sensitivity sens;
    int ret = gkb_get_sensitivity(&sens);
    if (ret < 0) {
        return ret;
    }

    *info = (pyuron_gesture_GestureInfo)pyuron_gesture_GestureInfo_init_zero;
    info->id = id;
    strncpy(info->name, "dynamic", sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    info->tick          = sens.tick;
    info->wait_ms       = sens.wait_ms;
    info->tap_ms        = sens.tap_ms;
    info->threshold     = sens.threshold;
    info->max_threshold = sens.max_threshold;
    return 0;
}

static int handle_get_count(const pyuron_gesture_GetCountRequest *req,
                            pyuron_gesture_Response *resp) {
    ARG_UNUSED(req);
    resp->which_response_type = pyuron_gesture_Response_get_count_tag;
    // Dynamic processor is a global singleton; always report count=1.
    resp->response_type.get_count.count = 1;
    return 0;
}

static int handle_get(const pyuron_gesture_GetGestureRequest *req,
                      pyuron_gesture_Response *resp) {
    resp->which_response_type = pyuron_gesture_Response_get_tag;
    resp->response_type.get = (pyuron_gesture_GetGestureResponse)
        pyuron_gesture_GetGestureResponse_init_zero;
    resp->response_type.get.has_gesture = true; // nanopb: submessage presence flag
    return fill_gesture_info(req->id, &resp->response_type.get.gesture);
}

static int handle_set_param(const pyuron_gesture_SetParamRequest *req,
                            pyuron_gesture_Response *resp) {
    if (req->param < pyuron_gesture_Param_PARAM_TICK ||
        req->param > pyuron_gesture_Param_PARAM_MAX_THRESHOLD) {
        return -EINVAL;
    }

    LOG_DBG("set gesture id=%u param=%d value=%d", req->id, (int)req->param, (int)req->value);

    int ret = gkb_set_param((enum gkb_param)req->param, req->value);
    if (ret < 0) {
        return ret;
    }
    // Persist so the tuned value survives reboot.
    gkb_save_param((enum gkb_param)req->param, req->value);

    resp->which_response_type = pyuron_gesture_Response_set_param_tag;
    resp->response_type.set_param = (pyuron_gesture_SetParamResponse)
        pyuron_gesture_SetParamResponse_init_zero;
    resp->response_type.set_param.has_gesture = true;
    return fill_gesture_info(req->id, &resp->response_type.set_param.gesture);
}

#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND)
static int fill_binding_info(uint32_t id, uint32_t dir, pyuron_gesture_BindingInfo *info) {
    uint16_t bid = 0;
    int32_t p1 = 0, p2 = 0;
    int ret = zip_keybind_get_binding(id, dir, &bid, &p1, &p2);
    if (ret < 0) {
        return ret;
    }
    *info = (pyuron_gesture_BindingInfo)pyuron_gesture_BindingInfo_init_zero;
    info->id = id;
    info->dir = dir;
    info->behavior_id = bid;
    info->param1 = p1;
    info->param2 = p2;
    return 0;
}

static int handle_get_binding(const pyuron_gesture_GetBindingRequest *req,
                              pyuron_gesture_Response *resp) {
    resp->which_response_type = pyuron_gesture_Response_get_binding_tag;
    resp->response_type.get_binding = (pyuron_gesture_GetBindingResponse)
        pyuron_gesture_GetBindingResponse_init_zero;
    resp->response_type.get_binding.has_binding = true;
    return fill_binding_info(req->id, req->dir, &resp->response_type.get_binding.binding);
}

static int handle_set_binding(const pyuron_gesture_SetBindingRequest *req,
                              pyuron_gesture_Response *resp) {
    int ret = zip_keybind_set_binding(req->id, req->dir, (uint16_t)req->behavior_id,
                                      req->param1, req->param2);
    if (ret < 0) {
        return ret;
    }
    zip_keybind_save_binding(req->id, req->dir); // 再起動後も保持
    resp->which_response_type = pyuron_gesture_Response_set_binding_tag;
    resp->response_type.set_binding = (pyuron_gesture_SetBindingResponse)
        pyuron_gesture_SetBindingResponse_init_zero;
    resp->response_type.set_binding.has_binding = true;
    return fill_binding_info(req->id, req->dir, &resp->response_type.set_binding.binding);
}
#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND) */

static int handle_reset(const pyuron_gesture_ResetGestureRequest *req,
                        pyuron_gesture_Response *resp) {
    // Reset all sensitivity params to DT defaults and clear NVS.
    // req->id is ignored: there is one global dynamic processor.
    int ret = gkb_reset_sensitivity();
    if (ret < 0) {
        return ret;
    }

    resp->which_response_type = pyuron_gesture_Response_reset_tag;
    resp->response_type.reset = (pyuron_gesture_ResetGestureResponse)
        pyuron_gesture_ResetGestureResponse_init_zero;
    resp->response_type.reset.has_gesture = true;
    return fill_gesture_info(req->id, &resp->response_type.reset.gesture);
}

/* -----------------------------------------------------------------------
 * Dynamic layer-gesture handlers
 * ----------------------------------------------------------------------- */

static int fill_layer_binding_info(uint8_t layer, uint8_t dir,
                                   pyuron_gesture_LayerBindingInfo *info) {
    uint16_t bid = 0;
    int32_t p1 = 0, p2 = 0;
    int ret = gkb_layer_get_binding(layer, dir, &bid, &p1, &p2);
    if (ret < 0) return ret;

    /* read enabled flag via gkb_layer_at — scan configured list */
    bool enabled = false;
    int count = gkb_layer_count();
    for (int i = 0; i < count; i++) {
        uint8_t l; bool e;
        if (gkb_layer_at(i, &l, &e) == 0 && l == layer) { enabled = e; break; }
    }

    *info = (pyuron_gesture_LayerBindingInfo)pyuron_gesture_LayerBindingInfo_init_zero;
    info->layer       = layer;
    info->dir         = dir;
    info->behavior_id = bid;
    info->param1      = p1;
    info->param2      = p2;
    info->enabled     = enabled;
    return 0;
}

static int handle_get_layer_binding(const pyuron_gesture_GetLayerBindingRequest *req,
                                    pyuron_gesture_Response *resp) {
    resp->which_response_type = pyuron_gesture_Response_get_layer_binding_tag;
    resp->response_type.get_layer_binding =
        (pyuron_gesture_GetLayerBindingResponse)
        pyuron_gesture_GetLayerBindingResponse_init_zero;
    resp->response_type.get_layer_binding.has_binding = true;
    return fill_layer_binding_info((uint8_t)req->layer, (uint8_t)req->dir,
                                   &resp->response_type.get_layer_binding.binding);
}

static int handle_set_layer_binding(const pyuron_gesture_SetLayerBindingRequest *req,
                                    pyuron_gesture_Response *resp) {
    int ret = gkb_layer_set_binding((uint8_t)req->layer, (uint8_t)req->dir,
                                    (uint16_t)req->behavior_id, req->param1, req->param2);
    if (ret < 0) return ret;
    gkb_layer_save_binding((uint8_t)req->layer, (uint8_t)req->dir);

    resp->which_response_type = pyuron_gesture_Response_set_layer_binding_tag;
    resp->response_type.set_layer_binding =
        (pyuron_gesture_SetLayerBindingResponse)
        pyuron_gesture_SetLayerBindingResponse_init_zero;
    resp->response_type.set_layer_binding.has_binding = true;
    return fill_layer_binding_info((uint8_t)req->layer, (uint8_t)req->dir,
                                   &resp->response_type.set_layer_binding.binding);
}

static int handle_enable_layer(const pyuron_gesture_EnableLayerRequest *req,
                               pyuron_gesture_Response *resp) {
    int ret = gkb_layer_enable((uint8_t)req->layer);
    if (ret < 0) return ret;
    gkb_layer_save_enable((uint8_t)req->layer);
    resp->which_response_type = pyuron_gesture_Response_enable_layer_tag;
    resp->response_type.enable_layer.layer   = req->layer;
    resp->response_type.enable_layer.enabled = true;
    return 0;
}

static int handle_disable_layer(const pyuron_gesture_DisableLayerRequest *req,
                                pyuron_gesture_Response *resp) {
    int ret = gkb_layer_disable((uint8_t)req->layer);
    if (ret < 0) return ret;
    gkb_layer_save_enable((uint8_t)req->layer);
    resp->which_response_type = pyuron_gesture_Response_disable_layer_tag;
    resp->response_type.disable_layer.layer   = req->layer;
    resp->response_type.disable_layer.enabled = false;
    return 0;
}

static int handle_get_layer_count(const pyuron_gesture_GetLayerCountRequest *req,
                                  pyuron_gesture_Response *resp) {
    ARG_UNUSED(req);
    resp->which_response_type = pyuron_gesture_Response_get_layer_count_tag;
    resp->response_type.get_layer_count.count = (uint32_t)gkb_layer_count();
    return 0;
}

static int handle_get_configured_layer(const pyuron_gesture_GetConfiguredLayerRequest *req,
                                       pyuron_gesture_Response *resp) {
    uint8_t layer; bool enabled;
    int ret = gkb_layer_at((int)req->index, &layer, &enabled);
    if (ret < 0) return ret;
    resp->which_response_type = pyuron_gesture_Response_get_configured_layer_tag;
    resp->response_type.get_configured_layer.layer   = layer;
    resp->response_type.get_configured_layer.enabled = enabled;
    return 0;
}

/* -----------------------------------------------------------------------
 * Per-(layer, direction) sensitivity handlers
 * ----------------------------------------------------------------------- */

static int fill_layer_sens_info(uint8_t layer, uint8_t dir,
                                pyuron_gesture_LayerSensInfo *info) {
    struct gkb_dir_sensitivity s;
    int ret = gkb_layer_get_sens(layer, dir, &s);
    if (ret < 0) return ret;
    *info = (pyuron_gesture_LayerSensInfo)pyuron_gesture_LayerSensInfo_init_zero;
    info->layer     = layer;
    info->dir       = dir;
    info->tick      = s.tick;
    info->wait_ms   = s.wait_ms;
    info->threshold = s.threshold;
    return 0;
}

static int handle_get_layer_sens(const pyuron_gesture_GetLayerSensRequest *req,
                                 pyuron_gesture_Response *resp) {
    resp->which_response_type = pyuron_gesture_Response_get_layer_sens_tag;
    resp->response_type.get_layer_sens =
        (pyuron_gesture_GetLayerSensResponse)pyuron_gesture_GetLayerSensResponse_init_zero;
    resp->response_type.get_layer_sens.has_sens = true;
    return fill_layer_sens_info((uint8_t)req->layer, (uint8_t)req->dir,
                                &resp->response_type.get_layer_sens.sens);
}

static int handle_set_layer_sens(const pyuron_gesture_SetLayerSensRequest *req,
                                 pyuron_gesture_Response *resp) {
    if (req->param != pyuron_gesture_Param_PARAM_TICK &&
        req->param != pyuron_gesture_Param_PARAM_WAIT_MS &&
        req->param != pyuron_gesture_Param_PARAM_THRESHOLD) {
        return -EINVAL;
    }
    int ret = gkb_layer_set_sens((uint8_t)req->layer, (uint8_t)req->dir,
                                 (enum gkb_param)req->param, req->value);
    if (ret < 0) return ret;
    gkb_layer_save_sens((uint8_t)req->layer, (uint8_t)req->dir);

    resp->which_response_type = pyuron_gesture_Response_set_layer_sens_tag;
    resp->response_type.set_layer_sens =
        (pyuron_gesture_SetLayerSensResponse)pyuron_gesture_SetLayerSensResponse_init_zero;
    resp->response_type.set_layer_sens.has_sens = true;
    return fill_layer_sens_info((uint8_t)req->layer, (uint8_t)req->dir,
                                &resp->response_type.set_layer_sens.sens);
}

static int handle_reset_layer_sens(const pyuron_gesture_ResetLayerSensRequest *req,
                                   pyuron_gesture_Response *resp) {
    int ret = gkb_layer_reset_sens((uint8_t)req->layer, (uint8_t)req->dir);
    if (ret < 0) return ret;
    gkb_layer_save_sens((uint8_t)req->layer, (uint8_t)req->dir);

    resp->which_response_type = pyuron_gesture_Response_reset_layer_sens_tag;
    resp->response_type.reset_layer_sens =
        (pyuron_gesture_ResetLayerSensResponse)pyuron_gesture_ResetLayerSensResponse_init_zero;
    resp->response_type.reset_layer_sens.has_sens = true;
    return fill_layer_sens_info((uint8_t)req->layer, (uint8_t)req->dir,
                                &resp->response_type.reset_layer_sens.sens);
}

static bool gesture_rpc_handle_request(const zmk_custom_CallRequest *raw_request,
                                       pb_callback_t *encode_response) {
    pyuron_gesture_Response *resp =
        ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER_ALLOCATE(pyuron_gesture, encode_response);

    pyuron_gesture_Request req = pyuron_gesture_Request_init_zero;

    pb_istream_t req_stream =
        pb_istream_from_buffer(raw_request->payload.bytes, raw_request->payload.size);
    if (!pb_decode(&req_stream, pyuron_gesture_Request_fields, &req)) {
        LOG_WRN("Failed to decode gesture request: %s", PB_GET_ERROR(&req_stream));
        pyuron_gesture_ErrorResponse err = pyuron_gesture_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message), "Failed to decode request");
        resp->which_response_type = pyuron_gesture_Response_error_tag;
        resp->response_type.error = err;
        return true;
    }

    int rc = 0;
    switch (req.which_request_type) {
    case pyuron_gesture_Request_get_count_tag:
        rc = handle_get_count(&req.request_type.get_count, resp);
        break;
    case pyuron_gesture_Request_get_tag:
        rc = handle_get(&req.request_type.get, resp);
        break;
    case pyuron_gesture_Request_set_param_tag:
        rc = handle_set_param(&req.request_type.set_param, resp);
        break;
    case pyuron_gesture_Request_reset_tag:
        rc = handle_reset(&req.request_type.reset, resp);
        break;
#if IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND)
    case pyuron_gesture_Request_get_binding_tag:
        rc = handle_get_binding(&req.request_type.get_binding, resp);
        break;
    case pyuron_gesture_Request_set_binding_tag:
        rc = handle_set_binding(&req.request_type.set_binding, resp);
        break;
#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND) */
    case pyuron_gesture_Request_get_layer_binding_tag:
        rc = handle_get_layer_binding(&req.request_type.get_layer_binding, resp);
        break;
    case pyuron_gesture_Request_set_layer_binding_tag:
        rc = handle_set_layer_binding(&req.request_type.set_layer_binding, resp);
        break;
    case pyuron_gesture_Request_enable_layer_tag:
        rc = handle_enable_layer(&req.request_type.enable_layer, resp);
        break;
    case pyuron_gesture_Request_disable_layer_tag:
        rc = handle_disable_layer(&req.request_type.disable_layer, resp);
        break;
    case pyuron_gesture_Request_get_layer_count_tag:
        rc = handle_get_layer_count(&req.request_type.get_layer_count, resp);
        break;
    case pyuron_gesture_Request_get_configured_layer_tag:
        rc = handle_get_configured_layer(&req.request_type.get_configured_layer, resp);
        break;
    case pyuron_gesture_Request_get_layer_sens_tag:
        rc = handle_get_layer_sens(&req.request_type.get_layer_sens, resp);
        break;
    case pyuron_gesture_Request_set_layer_sens_tag:
        rc = handle_set_layer_sens(&req.request_type.set_layer_sens, resp);
        break;
    case pyuron_gesture_Request_reset_layer_sens_tag:
        rc = handle_reset_layer_sens(&req.request_type.reset_layer_sens, resp);
        break;
    default:
        LOG_WRN("Unsupported gesture request type: %d", req.which_request_type);
        rc = -1;
    }

    if (rc != 0) {
        pyuron_gesture_ErrorResponse err = pyuron_gesture_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message), "Failed to process request (%d)", rc);
        resp->which_response_type = pyuron_gesture_Response_error_tag;
        resp->response_type.error = err;
    }
    return true;
}
