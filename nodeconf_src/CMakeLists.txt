cmake_minimum_required (VERSION 2.6)
project (nodeconf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(SOURCES
	main.cpp
)

# add the binary tree to the search path for include files
include_directories (${PROJECT_BINARY_DIR})
include_directories (${DOCOPT_INCLUDE_DIR})
include_directories (${SERIAL_INCLUDE_DIR})
include_directories (${PROTOCOL_INCLUDE_DIR})

set(EXECUTABLE_OUTPUT_PATH ../)

# add the executable
add_executable (nodeconf ${SOURCES})
target_link_libraries( 
	nodeconf
	serial
	protocol
	docopt
)