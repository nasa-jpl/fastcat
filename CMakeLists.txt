cmake_minimum_required (VERSION 3.11)
project(fastcat
    DESCRIPTION "C++ EtherCAT Device Command & Control Library"
    VERSION 0.11.5
    LANGUAGES C CXX
    )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_TESTS "Option to build unit and device test programs" ON)
option(DISABLE_DEBUG_PRINTS "Option to disable debug prints, default off" OFF)

if(DISABLE_DEBUG_PRINTS)
    remove_definitions(-DDEBUG)
else(DISABLE_DEBUG_PRINTS)
    add_definitions(-DDEBUG)
endif(DISABLE_DEBUG_PRINTS)

# @TODO(kwehage,abrinkma) Clean up missing-field-initializers warnings
add_definitions(
    -Wall
    -Wextra
    -Wno-missing-field-initializers
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB_RECURSE includes RELATIVE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    )
foreach(include ${includes})
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${include}"
        "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${include}"
        COPYONLY
        )
endforeach()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config.h.in
    "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/config.h"
    )

####### Dependencies #######
find_package(YamlCpp REQUIRED 0.6.3)

include(FetchContent)
FetchContent_Declare(jsd
    GIT_REPOSITORY https://github.com/nasa-jpl/jsd.git
    GIT_TAG v2.1.2
    )
FetchContent_MakeAvailable(jsd)

####### Build #######
add_subdirectory(src)


####### Test Suite #######
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    if(BUILD_TESTS)
        enable_testing()
        include(CTest)
        add_subdirectory(test)
    else()
        message(
            STATUS
            "Building test suite disabled, enable testing by setting BUILD_TESTS=ON"
            )
    endif()
else()
    message(STATUS "Not building test suite when built as submodule")
endif()

####### Install ########
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_INSTALL_PREFIX "/opt/fastcat")
    install(DIRECTORY ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME} DESTINATION include)
    install(TARGETS fastcat DESTINATION lib)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindFastcat.cmake DESTINATION /opt/cmake)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindYamlCpp.cmake DESTINATION /opt/cmake)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

    ######### "doc" target #########
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_SOURCE_DIR}/.doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/.doxyfile)
        add_custom_target ( doc
            COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_CURRENT_BINARY_DIR}/.doxyfile"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Creating Doxygen Documentation"
            VERBATIM
            )
    endif()

    ######### "format" target #########
    find_program(FOUND_CLANG_FORMAT clang-format)
    if(FOUND_CLANG_FORMAT)
        FILE(GLOB_RECURSE C_FILES
            "${CMAKE_CURRENT_LIST_DIR}/src/*.cc"
            "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
            "${CMAKE_CURRENT_LIST_DIR}/src/*.h"
            "${CMAKE_CURRENT_LIST_DIR}/test/*.cc"
            "${CMAKE_CURRENT_LIST_DIR}/test/*.h")
        add_custom_target( format
            COMMAND clang-format -i -style=file ${C_FILES}
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
            COMMENT "Formatting all .c/.h files recursively under: ${CMAKE_CURRENT_LIST_DIR}"
            VERBATIM
            )
    endif()

else()

    message(STATUS "Not building format or doc targets when built as submodule")

endif()
