
aux_source_directory( . src)
add_library(dh SHARED ${src})
target_link_libraries(dh ${OpenCV_LIBS})