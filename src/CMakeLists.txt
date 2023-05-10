####### Fastcat COG Autocoding #######

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/autogen)

function(run_fcgen fc_input_file fc_output_file)
  add_custom_command(
    OUTPUT
      ${fc_output_file}
    COMMAND
      cog -d 
      -D cog_yaml_file=fcgen/templates/${fc_input_file}
      -D command_yaml=fcgen/fastcat_commands.yaml
      -D device_yaml=fcgen/fastcat_devices.yaml
      -D enum_yaml=fcgen/fastcat_enums.yaml
      -D manager_yaml=fcgen/fastcat_manager_config.yaml
      -o ${fc_output_file}
      fcgen/templates/${fc_input_file}
    DEPENDS
      fcgen/fastcat_commands.yaml
      fcgen/fastcat_devices.yaml
      fcgen/fastcat_enums.yaml
      fcgen/fastcat_manager_config.yaml
      fcgen/templates/${fc_input_file}
    WORKING_DIRECTORY
      ${CMAKE_CURRENT_LIST_DIR}
    COMMENT
      "fcgen: ${fc_input_file} -> ${fc_output_file}"
    VERBATIM
    )
endfunction()

run_fcgen("types.h.cog"            "${CMAKE_BINARY_DIR}/include/fastcat/types.h")
run_fcgen("device_includes.h.cog"  "${CMAKE_BINARY_DIR}/include/fastcat/device_includes.h")
run_fcgen("signal_handling.cc.cog" "${CMAKE_BINARY_DIR}/autogen/signal_handling.cc")
run_fcgen("commander.cc.cog"       "${CMAKE_BINARY_DIR}/autogen/commander.cc")

add_custom_target(fcgen DEPENDS
    ${CMAKE_BINARY_DIR}/include/fastcat/types.h
    ${CMAKE_BINARY_DIR}/include/fastcat/device_includes.h
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

