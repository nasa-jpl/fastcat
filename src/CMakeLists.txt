####### Fastcat COG Autocoding #######
add_custom_command(
    OUTPUT
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    COMMAND
    cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
    -o ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/types.h.cog
    DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/fastcat_types.yaml
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/types.h.cog
    WORKING_DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
    "running fcgen - types"
    VERBATIM
    )

add_custom_command(
    OUTPUT
    ${CMAKE_BINARY_DIR}/fastcat/autogen/signal_handling.cc
    COMMAND
    cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
    -o ${CMAKE_BINARY_DIR}/fastcat/autogen/signal_handling.cc
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/signal_handling.cc.cog
    DEPENDS
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/signal_handling.cc.cog
    WORKING_DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
    "running fcgen - signals"
    VERBATIM
    )

add_custom_command(
    OUTPUT
    ${CMAKE_BINARY_DIR}/fastcat/autogen/fastcat_devices/commander.cc
    COMMAND
    cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
    -o ${CMAKE_BINARY_DIR}/fastcat/autogen/fastcat_devices/commander.cc
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/commander.cc.cog
    DEPENDS
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_CURRENT_LIST_DIR}/fcgen/commander.cc.cog
    WORKING_DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
    "running fcgen - commander"
    VERBATIM
    )

add_custom_target(fcgen
    DEPENDS
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_BINARY_DIR}/fastcat/autogen/signal_handling.cc
    ${CMAKE_BINARY_DIR}/fastcat/autogen/fastcat_devices/commander.cc
    )

add_library(fastcat STATIC
    trap.c
    device_base.cc
    manager.cc
    yaml_parser.cc
    ${CMAKE_BINARY_DIR}/fastcat/autogen/signal_handling.cc
    transform_utils.cc

    jsd/actuator.cc
    jsd/actuator_fsm_helpers.cc
    jsd/egd.cc
    jsd/el3208.cc
    jsd/el3602.cc
    jsd/el2124.cc
    jsd/jed.cc
    jsd/ati_fts.cc

    jsd/actuator_offline.cc
    jsd/egd_offline.cc
    jsd/el2124_offline.cc
    jsd/el3208_offline.cc
    jsd/el3602_offline.cc
    jsd/jed_offline.cc
    jsd/ati_fts_offline.cc

    ${CMAKE_BINARY_DIR}/fastcat/autogen/fastcat_devices/commander.cc
    fastcat_devices/signal_generator.cc
    fastcat_devices/function.cc
    fastcat_devices/conditional.cc
    fastcat_devices/pid.cc
    fastcat_devices/saturation.cc
    fastcat_devices/schmitt_trigger.cc
    fastcat_devices/filter.cc
    fastcat_devices/fts.cc
    fastcat_devices/virtual_fts.cc
    fastcat_devices/faulter.cc
    )

target_include_directories(
    fastcat PUBLIC
    ${CMAKE_BINARY_DIR}/include
    ${SOEM_INCLUDE_DIRS}
    )

target_link_libraries(fastcat
    jsd-lib
    ${YAMLCPP_LIBRARY}
    )

add_dependencies(fastcat
    fcgen
    )

