cmake_minimum_required (VERSION 2.6)
project (atlas)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Armadillo REQUIRED)
# find_package(yaml-cpp REQUIRED)

SET(SOURCES
	main.cpp
	atlas_types.h
	config.cpp
	config.h
	parser.cpp
	parser.h
	tdoa.cpp
	tdoa.h
	coordinate_systems.cpp
	coordinate_systems.h
	logger.cpp
	logger.h
	reporter.cpp
	reporter.h
)

# add the binary tree to the search path for include files
include_directories (${PROJECT_BINARY_DIR})
include_directories (${DOCOPT_INCLUDE_DIR})
include_directories (${PROTOCOL_INCLUDE_DIR})
include_directories (${SERIAL_INCLUDE_DIR})

set(EXECUTABLE_OUTPUT_PATH ../)

# add the executable
add_executable (atlas ${SOURCES})
target_link_libraries( 
	atlas
	serial
	protocol
	yaml-cpp
	armadillo 
	docopt
)