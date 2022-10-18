cmake_minimum_required(VERSION 3.1)
project(Sparton_AHRS_M1_Driver VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Boost-Asio
find_package(rtac_asio REQUIRED)

add_library(sparton_ahrs_m1_driver src/sparton_ahrs_m1_driver.cpp)
target_include_directories(sparton_ahrs_m1_driver PUBLIC include)
target_link_libraries(sparton_ahrs_m1_driver PUBLIC rtac_asio)

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE sparton_ahrs_m1_driver)