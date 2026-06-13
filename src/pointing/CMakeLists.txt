# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND app PRIVATE input-processor-keybind.c)
target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND_DYNAMIC app PRIVATE input-processor-keybind-dynamic.c)
