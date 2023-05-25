# Example
add_executable(main main.cpp)
target_link_libraries(main PRIVATE sparton_ahrs_m1_driver)
target_include_directories(main PUBLIC ${SPARTON_M1_DRIVER_HEADERS})

find_package(Eigen3 REQUIRED)

add_executable(rotations rotations.cpp)
target_link_libraries(rotations PRIVATE sparton_ahrs_m1_driver Eigen3::Eigen)
target_include_directories(rotations PUBLIC ${SPARTON_M1_DRIVER_HEADERS})