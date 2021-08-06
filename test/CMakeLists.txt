set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR})

add_executable(main main.cpp)
