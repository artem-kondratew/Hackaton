//
// Created by user on 3/18/23.
//

#include "Gservo.h"


Gservo::Gservo(uint8_t _id) {
    id = _id;
    goal = 0;
    angle = 0;
    speed = 0;
    torque = 0;
    is_moving = 0;
    x = 0;
    y = 0;
    z = 0;
}


uint8_t Gservo::get_id() const {
    return id;
}


uint16_t Gservo::get_goal() const {
    return goal;
}


uint16_t Gservo::get_angle() const {
    return angle;
}


uint16_t Gservo::get_speed() const {
    return speed;
}


uint16_t Gservo::get_torque() const {
    return torque;
}


uint16_t Gservo::get_is_moving() const {
    return is_moving;
}


uint16_t Gservo::get_x(){
    return x;
}


uint16_t Gservo::get_y(){
    return y;
}


uint16_t Gservo::get_z(){
    return z;
}


uint16_t Gservo::get_q0() {
    return q0;
}


uint16_t Gservo::get_q1() {
    return q1;
}


uint16_t Gservo::get_q2() {
    return q2;
}


void Gservo::set_goal(uint8_t _goal1, uint8_t _goal2) {
    goal = _goal1 * 100 + _goal2;
}


void Gservo::set_angle(uint8_t _angle1, uint8_t _angle2) {
    angle = _angle1 * 100 + _angle2;
}


void Gservo::set_speed(uint8_t _speed1, uint8_t _speed2) {
    speed = _speed1 * 100 + _speed2;
}


void Gservo::set_torque(uint8_t _torque1, uint8_t _torque2) {
    torque = _torque1 * 100 + _torque2;
}


void Gservo::set_is_moving(uint8_t _is_moving) {
    is_moving = _is_moving;
}


void Gservo::set_x(uint8_t _x1, uint8_t _x2, bool x_sign) {
    x = (_x1 * 100 + _x2);
    x = (x_sign) ? x : -x;
}


void Gservo::set_y(uint8_t _y1, uint8_t _y2, bool y_sign) {
    y = (_y1 * 100 + _y2);
    y = (y_sign) ? y : -y;
}


void Gservo::set_z(uint8_t _z1, uint8_t _z2, bool z_sign) {
    z = (_z1 * 100 + _z2);
    z = (z_sign) ? z : -z;
}


void Gservo::set_q0(uint8_t _q01, uint8_t _q02) {
    q0 = _q01 * 100 + _q02;
}


void Gservo::set_q1(uint8_t _q11, uint8_t _q12) {
    q1 = _q11 * 100 + _q12;
}


void Gservo::set_q2(uint8_t _q21, uint8_t _q22) {
    q2 = _q21 * 100 + _q22;
}
