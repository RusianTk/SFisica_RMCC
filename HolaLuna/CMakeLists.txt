cmake_minimum_required(VERSION 3.26)
project(HolaLuna)

set(CMAKE_CXX_STANDARD 17)

add_executable(HolaLuna main.cpp)

find_package(box2d CONFIG REQUIRED)
target_link_libraries(HolaLuna PRIVATE box2d::box2d)