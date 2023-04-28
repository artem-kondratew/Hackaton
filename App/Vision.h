//
// Created by user on 4/28/23.
//

#ifndef HACKATON_VISION_H
#define HACKATON_VISION_H


#include <unistd.h>
//#include <opencv2/opencv.hpp>
#include "Connect.h"


class Vision {
private:
    inline static bool processing_flag = false;
public:
    static void processing();
    static void start_processing();
    static void stop_processing();
    static bool is_processing();
};


#endif //HACKATON_VISION_H
