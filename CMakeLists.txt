cmake_minimum_required(VERSION 3.0)

project(dh)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(src)
add_subdirectory(test)
