cmake_minimum_required(VERSION 3.26)
project(HolaPelota)

set(CMAKE_CXX_STANDARD 17)

add_executable(HolaPelota main.cpp)

find_package(box2d CONFIG REQUIRED)
target_link_libraries(HolaPelota PRIVATE box2d::box2d)

find_package(SFML COMPONENTS system window graphics CONFIG REQUIRED)
target_link_libraries(HolaPelota PRIVATE sfml-system sfml-network sfml-graphics sfml-window)