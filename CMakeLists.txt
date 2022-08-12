cmake_minimum_required(VERSION 3.10)

# set the project name
project(hopper_low_level)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3")

find_package(Eigen3 REQUIRED)

add_executable(hopper src/PC_testing_1D.cpp)
target_link_libraries(hopper Eigen3::Eigen)
