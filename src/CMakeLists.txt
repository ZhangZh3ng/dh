
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
set(CMAKE_CXX_STANDARD "-std=c++11")
include_directories("~/桌面/optmProject/ceres-solver-master")
include_directories("~/桌面/optmProject/ceres-solver-master/examples")