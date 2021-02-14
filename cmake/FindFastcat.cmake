# This module defines
#  FASTCAT_FOUND, true if found
#  FASTCAT_LIBRARIES expression containing libraries
#  FASTCAT_INCLUDE_DIRS, expression containig headers

include(FindPackageHandleStandardArgs)
if(NOT FASTCAT_FOUND)

  find_package(JSD REQUIRED)
  find_package(YamlCpp REQUIRED)


  find_library(FASTCAT_LIBRARIES 
    NAME fastcat
    PATHS 
    /opt/fastcat
    PATH_SUFFIXES lib)

  list(APPEND FASTCAT_LIBRARIES 
    ${JSD_LIBRARIES}
    ${YAMLCPP_LIBRARY}
  )

  find_path(FASTCAT_INCLUDE_DIRS 
    NAME fastcat/fastcat.h 
    PATHS
    /opt/fastcat
    PATH_SUFFIXES include)

  list(APPEND FASTCAT_INCLUDE_DIRS 
    ${JSD_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIR}
    )

  set(FASTCAT_VERSION 0.4.0)

  find_package_handle_standard_args(FASTCAT 
    REQUIRED_VARS FASTCAT_LIBRARIES FASTCAT_INCLUDE_DIRS
    VERSION_VAR FASTCAT_VERSION
    FAIL_MESSAGE "Find FASTCAT unsuccessful")

  if(FASTCAT_FOUND)
    message (STATUS "FASTCAT_INCLUDE_DIRS = ${FASTCAT_INCLUDE_DIRS}")
    message (STATUS "FASTCAT_LIBRARIES = ${FASTCAT_LIBRARIES}")
  else(FASTCAT_FOUND)
    if(FASTCAT_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find FASTCAT!")
    endif(FASTCAT_FIND_REQUIRED)
  endif(FASTCAT_FOUND)

endif(NOT FASTCAT_FOUND)
