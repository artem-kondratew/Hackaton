//
// Created by user on 4/27/23.
//

#ifndef HACKATON_BEEP_H
#define HACKATON_BEEP_H


#include <fstream>
#include "Connect.h"


class Beep {
private:
    inline static std::ifstream file;
public:
    static void open();
    static void check();
};


void Beep::open() {

}


void Beep::check() {
    file.open(".beep");
    if (!file.is_open()) {
        return;
    }

    bool flag = false;

    while (!file.eof()) {
        file >> flag;
    }
    if (flag) {
        Connect::beep();
        file.close();
        return;
    }
}


#endif //HACKATON_BEEP_H
