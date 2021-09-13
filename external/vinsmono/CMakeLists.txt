
aux_source_directory( . src)
add_library(vinsmono SHARED ${src})

target_link_libraries(vinsmono ${OpenCV_LIBS})