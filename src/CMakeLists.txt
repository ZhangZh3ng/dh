
aux_source_directory( . src)
add_library(dh SHARED ${src})

# add_library(dh SHARED
#             geometry.h
#             imu.h
#             mywrite.h
#             parameter.h
#             type.h
#             unit.h
#             utils.h
#             wgs84.h)

# add_executable(dh_exe 
#             parameter.cpp
#             trajectory_generator.cpp
#             mywrite.cpp)

# add_executable(dh_exe 
#             parameter.cpp
#             trajectory_generator.cpp)