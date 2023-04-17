####### Fastcat COG Autocoding #######

add_custom_command(
    OUTPUT
      ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    COMMAND
      cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
      -o ${CMAKE_BINARY_DIR}/include/fastcat/types.h
      fcgen/types.h.cog
    DEPENDS
      fcgen/fastcat_types.yaml
      fcgen/types.h.cog
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
      "running fcgen - types.h"
    VERBATIM
    )

add_custom_command(
    OUTPUT
      ${CMAKE_BINARY_DIR}/include/fastcat/device_includes.h
    COMMAND
      cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
      -o ${CMAKE_BINARY_DIR}/include/fastcat/device_includes.h
      fcgen/device_includes.h.cog
    DEPENDS
      fcgen/fastcat_types.yaml
      fcgen/device_includes.h.cog
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
      "running fcgen - device_includes.h"
    VERBATIM
    )

add_custom_command(
    OUTPUT
      ${CMAKE_BINARY_DIR}/autogen/signal_handling.cc
    COMMAND
      cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
      -o ${CMAKE_BINARY_DIR}/autogen/signal_handling.cc
      fcgen/signal_handling.cc.cog
    DEPENDS
      ${CMAKE_BINARY_DIR}/include/fastcat/types.h
      fcgen/signal_handling.cc.cog
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
      "running fcgen - signal_handling.cc"
    VERBATIM
    )

add_custom_command(
    OUTPUT
      ${CMAKE_BINARY_DIR}/autogen/commander.cc
    COMMAND
      cog -d -D cog_yaml_file=fcgen/fastcat_types.yaml
      -o ${CMAKE_BINARY_DIR}/autogen/commander.cc
      fcgen/commander.cc.cog
    DEPENDS
      ${CMAKE_BINARY_DIR}/include/fastcat/types.h
      fcgen/commander.cc.cog
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
      "running fcgen - commander.cc"
    VERBATIM
    )

add_custom_target(fcgen
    DEPENDS
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_BINARY_DIR}/include/fastcat/device_includes.h
    ${CMAKE_BINARY_DIR}/autogen/signal_handling.cc
    ${CMAKE_BINARY_DIR}/autogen/commander.cc
    )

add_library(fastcat STATIC
    ${CMAKE_BINARY_DIR}/autogen/signal_handling.cc
    ${CMAKE_BINARY_DIR}/autogen/commander.cc
    trap.c
    device_base.cc
    manager.cc
    yaml_parser.cc
    transform_utils.cc
    jsd/jsd_device_base.cc

    # JSD Devices 
    jsd/actuator.cc
    jsd/actuator_fsm_helpers.cc
    jsd/ati_fts.cc
    jsd/egd.cc
    jsd/el3208.cc
    jsd/el3602.cc
    jsd/el2124.cc
    jsd/el4102.cc
    jsd/el3162.cc
    jsd/el3104.cc
    jsd/el3202.cc
    jsd/el3318.cc
    jsd/gold_actuator.cc
    jsd/ild1900.cc
    jsd/platinum_actuator.cc

    # Offline Devices
    jsd/ati_fts_offline.cc
    jsd/egd_offline.cc
    jsd/el2124_offline.cc
    jsd/el4102_offline.cc
    jsd/el3208_offline.cc
    jsd/el3602_offline.cc
    jsd/el3162_offline.cc
    jsd/el3104_offline.cc
    jsd/el3202_offline.cc
    jsd/el3318_offline.cc
    jsd/ild1900_offline.cc
    jsd/gold_actuator_offline.cc
    jsd/platinum_actuator_offline.cc

    # Fastcat Devices
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
    fastcat_devices/linear_interpolation.cc
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

