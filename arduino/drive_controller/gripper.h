#ifndef DRIVE_CONTROLLER_GRIPPER_H
#define DRIVE_CONTROLLER_GRIPPER_H


#include <Servo.h>


namespace Gripper {
    Servo grab_servo;
    Servo pitch_servo;
  
    uint8_t grab_pin_ = 9;
    uint8_t pitch_pin_ = 10;
  
    float pitch_min_ = 0;
    float pitch_max_ = 100;
    float grab_min_ = 40;
    float grab_max_ = 180;

    float default_pitch_ = 5;
    float default_grab_ = 40;

    void init();
    void set_angles(float grab, float pitch);
    void spin();
}


void Gripper::init() {
    grab_servo.attach(grab_pin_);
    pitch_servo.attach(pitch_pin_);
    grab_servo.write(default_grab_);
    pitch_servo.write(default_pitch_);
}


void Gripper::set_angles(float grab, float pitch) {
    grab_servo.write(grab);
    pitch_servo.write(pitch);
}


#endif // DRIVE_CONTROLLER_GRIPPER_H
