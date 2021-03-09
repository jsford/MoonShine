cmake_minimum_required(VERSION 3.1)

project(MoonShine)

add_subdirectory(ext)

add_executable(${PROJECT_NAME} src/moonshine.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE nanoplan fmt::fmt)
