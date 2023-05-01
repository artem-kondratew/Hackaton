//
// Created by user on 4/28/23.
//

#include "Vision.h"


void Vision::processing() {
    while (true) {
        if (!processing_flag) {
            continue;
        }
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
