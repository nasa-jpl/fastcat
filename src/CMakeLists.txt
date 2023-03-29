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

# Build a library regardless of BUILD_TESTS flag
set(FASTCAT_SOURCES
)

macro(create_fastcat_lib)
  message("creating library: ${ARGV0}")
  add_library(${ARGV0} STATIC
      ${PROJECT_SOURCE_DIR}/src/trap.c
      ${PROJECT_SOURCE_DIR}/src/device_base.cc
      ${PROJECT_SOURCE_DIR}/src/manager.cc
      ${PROJECT_SOURCE_DIR}/src/yaml_parser.cc
      ${CMAKE_BINARY_DIR}/fastcat/autogen/signal_handling.cc
      ${PROJECT_SOURCE_DIR}/src/transform_utils.cc

      ${PROJECT_SOURCE_DIR}/src/jsd/jsd_device_base.cc

      ${PROJECT_SOURCE_DIR}/src/jsd/actuator.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/actuator_fsm_helpers.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/egd.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3208.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3602.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el2124.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el4102.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3162.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3104.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3202.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3318.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/ild1900.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/jed0101.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/jed0200.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/ati_fts.cc

      ${PROJECT_SOURCE_DIR}/src/jsd/actuator_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/egd_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el2124_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el4102_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3208_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3602_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3162_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3104_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3202_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/el3318_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/ild1900_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/jed0101_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/jed0200_offline.cc
      ${PROJECT_SOURCE_DIR}/src/jsd/ati_fts_offline.cc

      ${CMAKE_BINARY_DIR}/fastcat/autogen/fastcat_devices/commander.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/signal_generator.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/function.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/conditional.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/pid.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/saturation.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/schmitt_trigger.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/filter.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/fts.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/virtual_fts.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/faulter.cc
      ${PROJECT_SOURCE_DIR}/src/fastcat_devices/linear_interpolation.cc
      )
  target_include_directories(${ARGV0} PUBLIC
      ${CMAKE_BINARY_DIR}/include
      ${SOEM_INCLUDE_DIRS}
      )
  
  target_link_libraries(${ARGV0}
      jsd-lib
      ${YAMLCPP_LIBRARY}
      )
  
  add_dependencies(${ARGV0}
      fcgen
      )
endmacro()

create_fastcat_lib(fastcat)
target_compile_definitions(fastcat PUBLIC PROTECTED=protected PRIVATE=private)
