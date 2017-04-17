##kipochen
##2016.7.27

cmake_minimum_required(VERSION 2.8 FATAL_ERROR) 
SET(CMAKE_BUILD_TYPE "Release")  
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fPIC")  
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")  
project(dmp)
include_directories(.)
link_directories(/usr/local/lib) 
aux_source_directory(. DIR_SRCS)

add_executable(exe ${DIR_SRCS}) 
target_link_libraries(exe lwpr) 
#add_library(dmp SHARED ${DIR_SRCS}) 
#add_library(dmp STATIC ${DIR_SRCS}) 



