# Example
add_executable(main main.cpp)
target_link_libraries(main PRIVATE sparton_ahrs_m1_driver)
target_include_directories(main PUBLIC ${SPARTON_M1_DRIVER_HEADERS})