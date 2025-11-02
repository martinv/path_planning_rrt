# list(APPEND geo_sources
# 	include/Point.hpp
# )

# add_library(geo SHARED ${geo_sources})

# MESSAGE("DEBUG: ${CMAKE_SOURCE_DIR}")
# target_include_directories(geo PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
# )

add_library(geo INTERFACE)
target_include_directories(geo INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
					 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_include_directories(geo INTERFACE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/rrt>
)

set_target_properties(geo PROPERTIES VERSION 0.1)
