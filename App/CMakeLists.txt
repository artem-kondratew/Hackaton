cmake_minimum_required(VERSION 3.18)
project(Hackaton)

set(CMAKE_CXX_STANDARD 17)

add_executable(Hackaton main.cpp ../Arduino/Config.h graphics.h Connect.h Connect.cpp str.h str.cpp Gservo.h Gservo.cpp History.h List.h Exception.h Vision.h Vision.cpp Header.h)

target_link_libraries(${PROJECT_NAME} ncurses)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Hackaton ${OpenCV_LIBS})
