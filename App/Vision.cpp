//
// Created by user on 4/28/23.
//

#include "Vision.h"


void Vision::init() {
    cap = cv::VideoCapture("v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink");
    if (!cap.isOpened()) {
        exit(-1);
    }

    int fourcc = 0;
    int fps = 30;
    cv::Size size(1280, 720);
    bool is_color = true;
    std::string pipeline = "appsrc ! videoconvert ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=4000";

    writer = cv::VideoWriter(pipeline, fourcc, fps, size, is_color);
    if (!writer.isOpened()) {
        exit(-1);
    }
}


void Vision::processing() {
    cv::Mat frame;
    while (true) {
        if (!processing_flag) {
            continue;
        }
        cap.read(frame);
        writer.write(frame);
    }
}


void Vision::start_processing() {
    processing_flag = true;
}


void Vision::stop_processing() {
    processing_flag = false;
}


bool Vision::is_processing() {
    return processing_flag;
}
