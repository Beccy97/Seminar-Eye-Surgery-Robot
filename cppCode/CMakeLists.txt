cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})

find_package(Eigen3)
find_package(Threads REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

add_library(eye.lib src/extApi.c src/extApiPlatform.c src/Jacobian_Pseudoinverse.cpp src/Forward_Kinematics.cpp)

add_executable(EyeSurgery src/connect.cpp)

target_link_libraries(EyeSurgery Threads::Threads)
target_link_libraries(EyeSurgery eye.lib)
