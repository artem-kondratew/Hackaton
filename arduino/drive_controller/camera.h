#ifndef DRIVE_CONTROLLER_CAMERA_H
#define DRIVE_CONTROLLER_CAMERA_H


#include <Servo.h>


namespace Camera {
    Servo yaw_servo;
    Servo pitch_servo;

    bool changed = false;

    uint8_t yaw_pin_ = 9;
    uint8_t pitch_pin_ = 10;
    
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
};


void Camera::init() {
    yaw_servo.write(default_yaw_);
    pitch_servo.write(default_pitch_);
    yaw_servo.attach(yaw_pin_);
    pitch_servo.attach(pitch_pin_);
}


void Camera::set_angles(float yaw, float pitch) {
    set_yaw(yaw);
    set_pitch(pitch);
    changed = true;
}


void Camera::spin() {
    if (!changed) {
        return;
    }
    yaw_servo.write(yaw_);
    pitch_servo.write(pitch_);
    changed = false;
}


#endif // DRIVE_CONTROLLER_CAMERA_H
