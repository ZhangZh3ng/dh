
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# add_library(dh_core SHARED
#             core.hpp)

# target_link_libraries(dh_core ${thirdparty_libraries})