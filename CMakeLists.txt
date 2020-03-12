cmake_minimum_required(VERSION 3.15)
project(libfranka
  VERSION 0.7.1
  LANGUAGES CXX
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)
#//------------add by G Alexander-------------//
#change to c++17
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#//------------add by G Alexander-------------//
#set default build type
set(default_build_type "Release")
message(STATUS "Setting build type to '${default_build_type}'")
set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS  "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin" OR ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
#//------------add by G Alexander-------------//
#include /usr/local on Linux/OSX LDFLAGS and CXXFLAGS
INCLUDE_DIRECTORIES(/usr/local/include)
LINK_DIRECTORIES(/usr/local/lib)
endif()

#//------------add by G Alexander-------------//
#additions for mac OSX
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_MACOSX_RPATH 1)
  set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "Build architectures for OSX: x86_64" FORCE)
  set(CMAKE_CXX_FLAGS_DEBUG "-DAPPLE -g" CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_MINSIZEREL "-DAPPLE -Os -DNDEBUG" CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_RELEASE "-DAPPLE -DNDEBUG" CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DAPPLE -g" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS_DEBUG "-DAPPLE -g" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS_MINSIZEREL "-DAPPLE -Os -DNDEBUG" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS_RELEASE "-DAPPLE -DNDEBUG" CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "-DAPPLE -g" CACHE STRING "" FORCE)
  set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/local/lib:/opt/local/lib)
  INCLUDE_DIRECTORIES(/opt/local/include)
  LINK_DIRECTORIES(/opt/local/lib)
endif()

#//------------add by G Alexander-------------//
#needs specific version of python
find_package( PythonInterp 2.7 REQUIRED )

if(MSVC)
  add_compile_options(/W0)
else()
#//------------add by G Alexander-------------//
#add pedantic for unix type compilers
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#//------------add by G Alexander-------------//
#change debug libs to "d"
SET(CMAKE_DEBUG_POSTFIX "d")

set(THIRDPARTY_SOURCES_DIR "${CMAKE_SOURCE_DIR}/3rdparty" CACHE PATH
  "Directory for third-party sources")

## Dependencies
#//------------add by G Alexander-------------//
#add Util
find_package(Poco REQUIRED COMPONENTS Net Foundation Util)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

## Build options
option(STRICT "Treat warnings as errors" OFF)
if(STRICT)
  if(MSVC)
    add_compile_options(/WX)
  else()
    add_compile_options(-Werror)
  endif()
endif()

option(BUILD_COVERAGE "Build with code coverage" OFF)
if(BUILD_COVERAGE)
  add_compile_options(--coverage)
  # Unfortunately, there is no add_link_options...
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

## Submodules
add_subdirectory(common)

## Library
add_library(franka SHARED
  src/control_loop.cpp
  src/control_tools.cpp
  src/control_types.cpp
  src/duration.cpp
  src/errors.cpp
  src/exception.cpp
  src/gripper.cpp
  src/gripper_state.cpp
  src/library_downloader.cpp
  src/library_loader.cpp
  src/load_calculations.cpp
  src/log.cpp
  src/logger.cpp
  src/lowpass_filter.cpp
  src/model.cpp
  src/model_library.cpp
  src/network.cpp
  src/rate_limiting.cpp
  src/robot.cpp
  src/robot_impl.cpp
  src/robot_state.cpp
  src/vacuum_gripper.cpp
  src/vacuum_gripper_state.cpp
)
add_library(Franka::Franka ALIAS franka)

set_target_properties(franka PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  VERSION ${libfranka_VERSION}
  SOVERSION ${libfranka_VERSION_MAJOR}.${libfranka_VERSION_MINOR} # Use minor version while we're at 0.x
  EXPORT_NAME Franka
)
target_compile_features(franka INTERFACE
  cxx_attribute_deprecated
  cxx_constexpr
  cxx_defaulted_functions
  cxx_deleted_functions
  cxx_generalized_initializers
  cxx_noexcept
  cxx_uniform_initialization
)
if(MSVC)
  target_compile_definitions(franka PUBLIC
    _USE_MATH_DEFINES # for M_PI in cmath
    NOMINMAX # avoid conflict with std::min
  )
endif()

target_include_directories(franka PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(franka PRIVATE
  Poco::Foundation
  Poco::Net
  Eigen3::Eigen3
  Threads::Threads
  libfranka-common
)

## Installation
include(GNUInstallDirs)
set(INSTALL_CMAKE_CONFIG_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/Franka)

install(TARGETS franka
  EXPORT FrankaTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

export(EXPORT FrankaTargets
  NAMESPACE Franka::
  FILE ${CMAKE_CURRENT_BINARY_DIR}/FrankaTargets.cmake
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(cmake/FrankaConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/FrankaConfigVersion.cmake
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)
install(EXPORT FrankaTargets
  NAMESPACE Franka::
  DESTINATION ${INSTALL_CMAKE_CONFIG_DIR}
)

## Subprojects

# Ignore find_package(Franka) in subprojects.
macro(find_package)
  if(NOT "${ARGV0}" STREQUAL Franka AND NOT DEFINED "${ARGV0}_FOUND")
    _find_package(${ARGV})
  endif()
endmacro()

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

option(BUILD_EXAMPLES "Build example code" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

option(BUILD_DOCUMENTATION "Build documentation" OFF)
if(BUILD_DOCUMENTATION)
  add_subdirectory(doc)
endif()

## Packaging
set(CPACK_PACKAGE_VENDOR "Franka Emika GmbH")
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_PACKAGE_VERSION ${libfranka_VERSION})
set(CPACK_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Debian
find_program(DPKG_PROG dpkg DOC "'dpkg' executable")
if(DPKG_PROG)
  execute_process(COMMAND ${DPKG_PROG} --print-architecture
    OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  # Change system name to use the correct architecture in file name
  set(CPACK_SYSTEM_NAME ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE})
endif()
# Debian versions require a dash
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}-1)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Franka Emika GmbH")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpoco-dev")
set(CPACK_DEBIAN_PACKAGE_CONFLICTS "ros-kinetic-libfranka, ros-melodic-libfranka")

include(CPack)

## Tools
file(GLOB_RECURSE SOURCES
  src/*.cpp
  examples/*.cpp
  common/*.cpp
)
file(GLOB_RECURSE HEADERS
  include/*.h
  src/*.h
  examples/*.h
  common/*.h
)
file(GLOB_RECURSE TEST_FILES
  test/*.h
  test/*.cpp
)
