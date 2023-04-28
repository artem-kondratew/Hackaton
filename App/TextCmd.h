//
// Created by user on 4/28/23.
//

#ifndef HACKATON_TEXTCMD_H
#define HACKATON_TEXTCMD_H


#include "Connect.h"


void Connect::stop() {
    connect_mutex.lock();
    resetCommand();
    setTask(STOP_TASK);
}


void Connect::moveForward() {
    connect_mutex.lock();
    resetCommand();
    setTask(MOVE_FORWARD_TASK);
}


void Connect::moveBackward() {
    connect_mutex.lock();
    resetCommand();
    setTask(MOVE_BACKWARD_TASK);
}


void Connect::turnRight() {
    connect_mutex.lock();
    resetCommand();
    setTask(TURN_RIGHT_TASK);
}


void Connect::turnLeft() {
    connect_mutex.lock();
    resetCommand();
    setTask(TURN_LEFT_TASK);
}


void Connect::push() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_PUSH_TASK);
    setValue(0);
}


void Connect::pop() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_POP_TASK);
}


void Connect::rise() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_RISE_TASK);
}


void Connect::drop() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_DROP_TASK);
}


void Connect::beep() {
    connect_mutex.lock();
    resetCommand();
    setTask(BEEP_TASK);
}


void Connect::rotate(uint8_t angle) {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_ROTATE_TASK);
    setValue(angle);
}


void Connect::shake() {
    connect_mutex.lock();
    resetCommand();
    setTask(SHAKE_TASK);
}


void Connect::visionBlink(int pin) {
    connect_mutex.lock();
    resetCommand();
    setTask(BLINK_TASK);
    setValue(pin);
}


void Connect::blink() {
    connect_mutex.lock();
    resetCommand();
    setTask(BLINK_TASK);
    setValue(7);
}


#endif //HACKATON_TEXTCMD_H
