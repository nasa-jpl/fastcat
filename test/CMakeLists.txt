message("Building tests, disable with BUILD_TESTS CMake option")

############################
## Simple C Test Programs ##
############################

include_directories(
    ${READLINE_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/include
    )

set(fastcat_test_libs
    fastcat
    m
    -lreadline
    )

add_executable(test_config test_config.cc)
target_link_libraries(test_config ${fastcat_test_libs})

add_executable(test_cli test_cli.cc)
target_link_libraries(test_cli ${fastcat_test_libs})

find_package(GTest)
if(GTEST_FOUND)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/device_tests)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/unit_tests)

    find_program(VALGRIND_EXECUTABLE NAMES valgrind)
    if(VALGRIND_EXECUTABLE)
        set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_EXECUTABLE})
        set(MEMORYCHECK_COMMAND_OPTIONS
            "--trace-children=yes --leak-check=full --track-origins=yes --error-exitcode=1")
        add_custom_target(
            memcheck
            ${CMAKE_CTEST_COMMAND} -D ExperimentalMemCheck \${ARGS}
            WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
            )
    else()
        message(WARNING "Memcheck target will not be built, valgrind was not found on this machine")
    endif()
else()

    message(WARNING "GTest not found, test suite will not be built")

endif()


