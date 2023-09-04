cmake_minimum_required(VERSION 2.8)

project(helloSLAM_Project)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Pangolin REQUIRED)
#指定Eigen的头文件目录，无需将程序链接到库上
#后续可以用find_package找到path
include_directories(/opt/homebrew/Cellar/eigen/3.4.0_1/include)
include_directories(${Pangolin_INCLUDE_DIRS})

add_library(libHello SHARED libHelloSLAM.cpp)

add_executable(helloSLAM_Program main.cpp)

target_link_libraries(helloSLAM_Program libHello)

target_link_libraries(helloSLAM_Program ${Pangolin_LIBRARIES})
