message("Building fastcat profile utilities")

include_directories(
    ${CMAKE_BINARY_DIR}/include
    ${YAMLCPP_INCLUDE_DIR}
    )

add_library(profile_utils STATIC profile_utils.cpp)
target_include_directories(profile_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(profile_utils fastcat ${YAMLCPP_LIBRARY} pthread rt)

add_executable(elmo_vel_profile elmo_vel_profile.cpp)
target_link_libraries(elmo_vel_profile profile_utils)

add_executable(elmo_pos_profile elmo_pos_profile.cpp)
target_link_libraries(elmo_pos_profile profile_utils)

option(AUTO_SETCAP "Run sudo setcap on profile utilities post-build" OFF)
if(AUTO_SETCAP)
    foreach(tgt elmo_vel_profile elmo_pos_profile)
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND sudo setcap cap_net_admin,cap_net_raw=eip $<TARGET_FILE:${tgt}>
            COMMENT "Setting capabilities on ${tgt}")
    endforeach()
endif()

####### Unit tests #######
# The profile utilities are unit tested independently of the top-level test
# suite so that everything for these tools lives under utils/.
if(BUILD_TESTS AND CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    find_package(GTest)
    find_package(Threads REQUIRED)
    if(GTEST_FOUND)
        # utils/ is added before the root enable_testing(), so opt in here
        enable_testing()
        add_executable(test_profile_utils test_profile_utils.cc)
        target_link_libraries(test_profile_utils PRIVATE
            ${GTEST_BOTH_LIBRARIES}
            Threads::Threads
            profile_utils
            )
        target_include_directories(test_profile_utils PUBLIC ${GTEST_INCLUDE_DIRS})
        add_test(NAME test_profile_utils COMMAND test_profile_utils)
    else()
        message(WARNING "GTest not found, profile utility tests will not be built")
    endif()
endif()
