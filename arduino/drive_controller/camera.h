#ifndef DRIVE_CONTROLLER_CAMERA_H
#define DRIVE_CONTROLLER_CAMERA_H


#include <Servo.h>


Servo camera_yaw_servo;
Servo camera_pitch_servo;


namespace Camera {
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
    camera_yaw_servo.write(default_yaw_);
    camera_pitch_servo.write(default_pitch_);
}


void Camera::set_angles(float yaw, float pitch) {
    set_yaw(yaw);
    set_pitch(pitch);
}


void Camera::spin() {
    camera_yaw_servo.write(yaw_);
    camera_pitch_servo.write(pitch_);
}


#endif // DRIVE_CONTROLLER_CAMERA_H
