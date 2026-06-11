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
#include <zmk/pointing/zip_keybind.h>
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

// Fill a protobuf GestureInfo from the live state of instance `id`.
static int fill_gesture_info(uint32_t id, pyuron_gesture_GestureInfo *info) {
    struct zip_keybind_info src;
    int ret = zip_keybind_get_info(id, &src);
    if (ret < 0) {
        return ret;
    }

    *info = (pyuron_gesture_GestureInfo)pyuron_gesture_GestureInfo_init_zero;
    info->id = id;
    strncpy(info->name, src.name ? src.name : "", sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';
    info->tick = src.tick;
    info->wait_ms = src.wait_ms;
    info->tap_ms = src.tap_ms;
    info->threshold = src.threshold;
    info->max_threshold = src.max_threshold;
    return 0;
}

static int handle_get_count(const pyuron_gesture_GetCountRequest *req,
                            pyuron_gesture_Response *resp) {
    ARG_UNUSED(req);
    resp->which_response_type = pyuron_gesture_Response_get_count_tag;
    resp->response_type.get_count.count = (uint32_t)zip_keybind_get_count();
    return 0;
}

static int handle_get(const pyuron_gesture_GetGestureRequest *req,
                      pyuron_gesture_Response *resp) {
    resp->which_response_type = pyuron_gesture_Response_get_tag;
    resp->response_type.get = (pyuron_gesture_GetGestureResponse)
        pyuron_gesture_GetGestureResponse_init_zero;
    return fill_gesture_info(req->id, &resp->response_type.get.gesture);
}

static int handle_set_param(const pyuron_gesture_SetParamRequest *req,
                            pyuron_gesture_Response *resp) {
    if (req->param < pyuron_gesture_Param_PARAM_TICK ||
        req->param > pyuron_gesture_Param_PARAM_MAX_THRESHOLD) {
        return -EINVAL;
    }

    LOG_DBG("set gesture id=%u param=%d value=%d", req->id, (int)req->param, (int)req->value);

    int ret = zip_keybind_set_param(req->id, (enum zip_keybind_param)req->param, req->value);
    if (ret < 0) {
        return ret;
    }

    resp->which_response_type = pyuron_gesture_Response_set_param_tag;
    resp->response_type.set_param = (pyuron_gesture_SetParamResponse)
        pyuron_gesture_SetParamResponse_init_zero;
    return fill_gesture_info(req->id, &resp->response_type.set_param.gesture);
}

static int handle_reset(const pyuron_gesture_ResetGestureRequest *req,
                        pyuron_gesture_Response *resp) {
    int ret = zip_keybind_reset(req->id);
    if (ret < 0) {
        return ret;
    }

    resp->which_response_type = pyuron_gesture_Response_reset_tag;
    resp->response_type.reset = (pyuron_gesture_ResetGestureResponse)
        pyuron_gesture_ResetGestureResponse_init_zero;
    return fill_gesture_info(req->id, &resp->response_type.reset.gesture);
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
