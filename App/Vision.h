//
// Created by user on 4/28/23.
//

#ifndef HACKATON_VISION_H
#define HACKATON_VISION_H


#include "header.h"
#include "Connect.h"


class Vision {
private:
    inline static cv::VideoCapture cap;
    inline static cv::VideoWriter writer;
    inline static bool processing_flag = false;
public:
    static void init();
    static void processing();
    static void start_processing();
    static void stop_processing();
    static bool is_processing();
};


#endif //HACKATON_VISION_H
