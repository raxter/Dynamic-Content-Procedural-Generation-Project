

# project name
PROJECT(PROC_GEN)


# 2.4.3 is actual minimum, but you have to add additional hacks
# uncomment this if you need 2.4.3 compatibility
# if(COMMAND cmake_policy)
#	 cmake_policy(SET CMP0003 NEW)
# endif(COMMAND cmake_policy)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# find and setup Qt4 for this project
FIND_PACKAGE(Qt4 REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/pal)

#ADD_LIBRARY(PAL_LIB SHARED ${CMAKE_SOURCE_DIR}/pal/lib/libpal.so)

# tell cmake to process CMakeLists.txt in these subdirs
SUBDIRS(src)



