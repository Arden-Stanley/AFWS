cmake_minimum_required(VERSION 4.2)

project(AFWSClient)

add_subdirectory(libs/wxWidgets-3.3.1/)

add_executable(AFWSClient src/main.cpp)

target_link_libraries(AFWSClient wx::net wx::core wx::base)