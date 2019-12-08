project(VisualOdometry)
cmake_minimum_required(VERSION 3.15.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17")

find_package(OpenCV REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
