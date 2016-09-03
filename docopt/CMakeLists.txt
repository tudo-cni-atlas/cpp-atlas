cmake_minimum_required(VERSION 2.6)

set(DOCOPT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DOCOPT_INCLUDE_DIR ${DOCOPT_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET docopt)
	message("docopt is already defined")
else()
	include_directories(${DOCOPT_INCLUDE_DIR})
	set(LIB_SRCS docopt_private.h docopt_util.h docopt_value.h docopt.cpp docopt.h)
	add_library(docopt STATIC ${LIB_SRCS})
endif()