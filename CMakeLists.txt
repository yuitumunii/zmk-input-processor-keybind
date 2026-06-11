add_subdirectory(src/pointing)

zephyr_include_directories(include)

# Custom Studio RPC for live gesture tuning. Needs the custom-subsystem support
# present in the cormoran custom-studio-protocol ZMK base.
if(CONFIG_ZMK_INPUT_PROCESSOR_KEYBIND_STUDIO_RPC)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/studio/gesture_handler.c)

    list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
    include(nanopb)
    set(NANOPB_GENERATE_CPP_APPEND_PATH TRUE)
    set(NANOPB_GENERATE_CPP_STANDALONE OFF)

    # Build the generated proto into its own library (adding generated sources
    # straight to the app target causes build-order issues; see template).
    zephyr_library()
    file(GLOB_RECURSE PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto)
    nanopb_generate_cpp(proto_srcs proto_hdrs RELPATH ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILES})
    target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
    target_sources(${ZEPHYR_CURRENT_LIBRARY} PRIVATE ${proto_srcs} ${proto_hdrs})

    target_include_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/proto)
    add_dependencies(app ${ZEPHYR_CURRENT_LIBRARY})
endif()
