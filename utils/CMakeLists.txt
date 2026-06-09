message("Building fastcat profile utilities")

include_directories(
    ${CMAKE_BINARY_DIR}/include
    ${YAMLCPP_INCLUDE_DIR}
    )

add_executable(elmo_vel_profile elmo_vel_profile.cpp)
target_link_libraries(elmo_vel_profile fastcat ${YAMLCPP_LIBRARY} pthread rt)

add_executable(elmo_pos_profile elmo_pos_profile.cpp)
target_link_libraries(elmo_pos_profile fastcat ${YAMLCPP_LIBRARY} pthread rt)

option(AUTO_SETCAP "Run sudo setcap on profile utilities post-build" OFF)
if(AUTO_SETCAP)
    foreach(tgt elmo_vel_profile elmo_pos_profile)
        add_custom_command(TARGET ${tgt} POST_BUILD
            COMMAND sudo setcap cap_net_admin,cap_net_raw=eip $<TARGET_FILE:${tgt}>
            COMMENT "Setting capabilities on ${tgt}")
    endforeach()
endif()
