cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(serial)
add_subdirectory(docopt)
add_subdirectory(protocol)

add_subdirectory(atlas_src)
add_subdirectory(nodeconf_src)