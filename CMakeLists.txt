cmake_minimum_required(VERSION 3.0)

project(dh)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/external )
include_directories( ${CMAKE_SOURCE_DIR}/src )

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(external/slam)