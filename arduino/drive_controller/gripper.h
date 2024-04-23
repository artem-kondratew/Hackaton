#ifndef DRIVE_CONTROLLER_GRIPPER_H
#define DRIVE_CONTROLLER_GRIPPER_H


#include <Servo.h>


Servo gripper_yaw_servo;
Servo gripper_pitch_servo;


namespace Gripper {
    float yaw_min_ = 0;
    float yaw_max_ = 255;
    float pitch_min_ = 0;
    float pitch_max_ = 255;

    float default_yaw_ = 100;
    float default_pitch_ = 100;

    float yaw_;
    float pitch_;

    void set_yaw(float yaw) {yaw_ = yaw;}
    void set_pitch(float pitch) {pitch_ = pitch;}

    void init();
    void set_angles(float yaw, float pitch);
    void spin();
}


void Gripper::init() {
    camera_yaw_servo.write(default_yaw_);
    camera_pitch_servo.write(default_pitch_);
}


void Gripper::set_angles(float yaw, float pitch) {
    set_yaw(yaw);
    set_pitch(pitch);
}


void Gripper::spin() {
    camera_yaw_servo.write(yaw_);
    camera_pitch_servo.write(pitch_);
}


#endif // DRIVE_CONTROLLER_GRIPPER_H
