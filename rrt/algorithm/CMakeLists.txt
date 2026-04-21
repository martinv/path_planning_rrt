add_library(algorithm INTERFACE)
target_include_directories(algorithm INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
					     "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_include_directories(algorithm INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/rrt>
)

set_target_properties(algorithm PROPERTIES VERSION 0.1)
