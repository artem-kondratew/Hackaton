#ifndef DRIVE_CONTROLLER_CAMERA_H
#define DRIVE_CONTROLLER_CAMERA_H


#include <Servo.h>


namespace Camera {
    Servo yaw_servo;
    Servo pitch_servo;

    uint8_t yaw_pin_ = 11;
    uint8_t pitch_pin_ = 3;
    
    float yaw_min_ = 0;
    float yaw_max_ = 180;
    float pitch_min_ = 40;
    float pitch_max_ = 180;

    float default_yaw_ = 100;
    float default_pitch_ = 100;

    void init();
    void set_angles(float yaw, float pitch);
}


void Camera::init() {
    yaw_servo.attach(yaw_pin_);
    pitch_servo.attach(pitch_pin_);
    yaw_servo.write(default_yaw_);
    pitch_servo.write(default_pitch_); 
}


void Camera::set_angles(float yaw, float pitch) {
    yaw_servo.write(yaw);
    pitch_servo.write(pitch); 
}


#endif // DRIVE_CONTROLLER_CAMERA_H
