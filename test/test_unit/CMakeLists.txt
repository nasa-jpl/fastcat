find_package(Threads REQUIRED)


set(TEST_SOURCES
    test_gold_actuator.cc
    test_commander.cc
    test_conditional.cc
    test_function.cc
    test_jsd_device_base.cc
    test_linear_interpolation.cc
    test_schmitt_trigger.cc
    test_signal_generator.cc
    test_transform_utils.cc
    test_trap.cc
    test_yaml_parser.cc
    test_fts.cc
    test_ati_fts.cc
    test_virtual_fts.cc
    )

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_EXECUTABLE ${TEST_SOURCE} NAME_WE)
    message(STATUS ${TEST_EXECUTABLE})
    add_executable(
        ${TEST_EXECUTABLE}
        ${TEST_SOURCE}
        )
    target_link_libraries(
        ${TEST_EXECUTABLE} PRIVATE
        ${GTEST_BOTH_LIBRARIES}
        Threads::Threads
        fastcat
        )
    target_include_directories(
        ${TEST_EXECUTABLE} PUBLIC
        ${GTEST_INCLUDE_DIRS}
        )
    add_test(
        NAME ${TEST_EXECUTABLE}
        COMMAND ${TEST_EXECUTABLE}
        )

endforeach()

