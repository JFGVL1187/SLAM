cmake_minimum_required(VERSION 2.8)

project(helloSLAM_Project)

set(CMAKE_BUILD_TYPE "Debug")

add_library(libHello SHARED libHelloSLAM.cpp)

add_executable(helloSLAM_Program main.cpp)

target_link_libraries(helloSLAM_Program libHello)