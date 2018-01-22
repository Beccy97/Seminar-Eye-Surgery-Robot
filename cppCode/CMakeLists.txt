set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_minimum_required(VERSION 3.5)

include_directories(include)

add_library(eye.lib src/extApi.c src/extApiPlatform.c)

add_executable(EyeSurgery src/connect.cpp)
