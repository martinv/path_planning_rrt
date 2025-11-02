list(APPEND PATH_PLANNING_INCLUDES)

add_subdirectory(geo)
add_subdirectory(utils)

list(APPEND PATH_PLANNING_SOURCES
     Main.cpp)

add_executable(main ${PATH_PLANNING_SOURCES})
target_link_libraries(main PRIVATE graaflib fmt geo)

enable_testing()
add_subdirectory(test)
