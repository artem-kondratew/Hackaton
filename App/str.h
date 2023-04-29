//
// Created by user on 3/17/23.
//

#ifndef MANIPULATOR_STR_H
#define MANIPULATOR_STR_H


#include "Header.h"


class str {
private:
    std::string string;
    int curs;

public:
    str();

    void push(uint8_t symbol, int index);
    void pop(int index);
    void reset();
    uint64_t size();
    const char* get();
    std::string get_str();
    void keyBackspace();
    void keyDelete();
    void setCurs(int CURS_X);
    int getCurs() const;
    void set(std::string _string);
};


#endif //MANIPULATOR_STR_H
