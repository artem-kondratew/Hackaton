#ifndef DRIVE_CONTROLLER_GRIPPER_H
#define DRIVE_CONTROLLER_GRIPPER_H


#include <Servo.h>


Servo gripper_yaw_servo;
Servo gripper_pitch_servo;


namespace Gripper {
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
    camera_yaw_servo.write(default_grab_);
    camera_pitch_servo.write(default_pitch_);
}


void Gripper::set_angles(float grab, float pitch) {
    set_yaw(grab);
    set_pitch(pitch);
}


void Gripper::spin() {
    camera_yaw_servo.write(grab_);
    camera_pitch_servo.write(pitch_);
}


#endif // DRIVE_CONTROLLER_GRIPPER_H
