add_library(utils INTERFACE)
target_include_directories(utils INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
				 	 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_include_directories(utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/graaflib/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/rrt>
)

set_target_properties(geo PROPERTIES VERSION 0.1)
