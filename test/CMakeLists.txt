set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR})
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# add_executable(main main2.cpp)
add_executable(read readfile.cpp)
add_executable(opt1 opt1.cpp)
add_executable(opt2 opt2.cpp)
add_executable(test_friend test_friend.cpp)
add_executable(t_tg test_trajectory_generator.cpp)
target_link_libraries(opt1 ${CERES_LIBRARIES})
target_link_libraries(opt2 ${CERES_LIBRARIES})

target_link_libraries(t_tg dh)