cmake_minimum_required(VERSION 3.14)

project(fastcat_test_app
  VERSION 0.0.0
  LANGUAGES C CXX
  DESCRIPTION "Fastcat test C++ app"
)

include(FetchContent)
FetchContent_Declare(
  fastcat
  GIT_REPOSITORY https://github.com/nasa-jpl/fastcat.git
  GIT_TAG apb-ut-visibility-declarations
)
FetchContent_MakeAvailable(fastcat)

add_executable(ftest
  ftest.cc
)

target_link_libraries(ftest fastcat)


# Test that the protected and private overrides work properly
#add_executable(ftest-invalid-access
#  ftest_invalid_access.cc
#)
#target_link_libraries(ftest-invalid-access fastcat)

# Test that the fastcat-test lib is not available to applications by accident
add_executable(ftest-invalid-access-lib
  ftest_invalid_access.cc
)
target_link_libraries(ftest-invalid-access-lib fastcat-test)
