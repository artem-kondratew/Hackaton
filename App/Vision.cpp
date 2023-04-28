//
// Created by user on 4/28/23.
//

#include "Vision.h"


void Vision::processing() {
    while (true) {
        if (!processing_flag) {
            continue;
        }
        Connect::blink(13);
        sleep(5);
    }
}


void Vision::start_processing() {
    processing_flag = true;
}


void Vision::stop_processing() {
    processing_flag = false;
}
