cmake_minimum_required(VERSION 3.18)
project(Hackaton)

set(CMAKE_CXX_STANDARD 20)

add_executable(Hackaton main.cpp ../Arduino/Config.h graphics.h Connect.h Connect.cpp str.h str.cpp Gservo.h Gservo.cpp History.h List.h Exception.h Beep.h)

set(CMAKE_CXX_FLAGS "-lncurses")