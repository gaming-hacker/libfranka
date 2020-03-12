cmake_minimum_required(VERSION 3.15)

add_library(libfranka-common INTERFACE)
set_target_properties(libfranka-common PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include
)
