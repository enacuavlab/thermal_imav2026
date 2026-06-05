cmake_minimum_required(VERSION 3.16)

project(thermal_mlx90640 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CLI11 CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZMQ REQUIRED IMPORTED_TARGET libzmq)

add_library(mlx90640_driver
    src/MLX90640_API.c
    src//MLX90640_I2C_Driver.c
)

target_include_directories(mlx90640_driver
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(mlx90640_driver
    PUBLIC m
)

add_executable(mlx90640_stats
    src/mlx90640_stats.cpp
)

target_link_libraries(mlx90640_stats
    PRIVATE
        mlx90640_driver
        CLI11::CLI11
        PkgConfig::LIBZMQ
)
