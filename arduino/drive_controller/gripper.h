#ifndef DRIVE_CONTROLLER_GRIPPER_H
#define DRIVE_CONTROLLER_GRIPPER_H


#include <Servo.h>


namespace Gripper {
    Servo grab_servo;
    Servo pitch_servo;

    bool changed = false;
  
    uint8_t grab_pin_ = 9;
    uint8_t pitch_pin_ = 10;
  
    float pitch_min_ = 0;
    float pitch_max_ = 100;
    float grab_min_ = 40;
    float grab_max_ = 180;

    float default_pitch_ = 0;
    float default_grab_ = 100;

    float grab_;
    float pitch_;

    void set_grab(float grab) {grab_ = grab;}
    void set_pitch(float pitch) {pitch_ = pitch;}

    void init();
    void set_angles(float grab, float pitch);
    void spin();
}


void Gripper::init() {
    grab_servo.write(default_grab_);
    pitch_servo.write(default_pitch_);
    grab_servo.attach(grab_pin_);
    pitch_servo.attach(pitch_pin_);
}


void Gripper::set_angles(float grab, float pitch) {
    set_grab(grab);
    set_pitch(pitch);
    changed = true;
}


void Gripper::spin() {
    if (!changed) {
        return;
    }
    grab_servo.write(grab_);
    pitch_servo.write(pitch_);
    changed = false;
}


#endif // DRIVE_CONTROLLER_GRIPPER_H
