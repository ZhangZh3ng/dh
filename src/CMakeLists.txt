
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# include_directories("~/桌面/optmProject/ceres-solver-master")
# include_directories("~/桌面/optmProject/ceres-solver-master/examples")

aux_source_directory( . src)

add_library(dh SHARED ${src})