cmake_minimum_required(VERSION 4.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(AFWSClient)

add_subdirectory(libs/wxWidgets-3.3.1/)

add_executable(AFWSClient src/App.cpp src/GUI.cpp)

target_link_libraries(AFWSClient wx::net wx::core wx::base)
