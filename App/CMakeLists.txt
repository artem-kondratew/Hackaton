cmake_minimum_required(VERSION 3.18)
project(Hackaton)

set(CMAKE_CXX_STANDARD 20)

add_executable(Hackaton main.cpp ../Arduino/Config.h graphics.h Connect.h Connect.cpp str.h str.cpp Gservo.h Gservo.cpp History.h List.h Exception.h Vision.h Vision.cpp TextCmd.h)

set(CMAKE_CXX_FLAGS "-lncurses")

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#add_executable(OCV main.cpp)
#target_link_libraries(OCV ${OpenCV_LIBS})