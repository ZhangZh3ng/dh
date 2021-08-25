cmake_minimum_required(VERSION 3.0)

project(dh)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)
add_subdirectory(src)
add_subdirectory(test)
