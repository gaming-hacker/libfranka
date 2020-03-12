find_package(Eigen3 CONFIG)
mark_as_advanced(FORCE Eigen3_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
  FOUND_VAR Eigen3_FOUND
  REQUIRED_VARS EIGEN3_INCLUDE_DIRS
)

#//------------add by G Alexander-------------//
#add diagnostic messages, Eigen3 install locations vary from OSX to Linux
if(Eigen3_FOUND)
  message (STATUS "Found components for Eigen3")
  message (STATUS "EIGEN3_INCLUDE_DIRS  = ${EIGEN3_INCLUDE_DIRS}")
endif()

if(NOT TARGET Eigen3::Eigen3)
  add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
    INTERFACE_COMPILE_DEFINITIONS "${EIGEN3_DEFINITIONS}"
  )
endif()
