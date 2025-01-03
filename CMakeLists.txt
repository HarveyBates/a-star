cmake_minimum_required(VERSION 3.26)

project(a_star C)

set(CMAKE_C_STANDARD 11)

find_package(SDL2 REQUIRED CONFIG REQUIRED COMPONENTS SDL2)

add_executable(${CMAKE_PROJECT_NAME} main.c)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SDL2::SDL2)
