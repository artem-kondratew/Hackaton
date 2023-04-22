
#ifndef Camera_h
#define Camera_h


#include <Servo.h>


Servo yaw_servo;  // Head servo D10
Servo pitch_servo;  // Bottom servo D11


class Camera{
public:
    static void init();
    static void pitch(int angle);
    static void yaw(int angle);
    static void setStartPosition();
};


void Camera::init() {
    yaw_servo.attach(10);
    pitch_servo.attach(11);
    setStartPosition();
}


void Camera::pitch(int angle) {
    if(angle > 180) {
        angle = 180;
    }
    if(angle < 40) {
        angle = 40;
    }
    pitch_servo.write(angle);
}


void Camera::yaw(int angle){
    if(angle > 180) {
        angle = 180;
    }
    if(angle < 0) {
        angle = 0;
    }
    yaw_servo.write(angle);
}


void Camera::setStartPosition() {
    yaw_servo.write(100);
    pitch_servo.write(100);
}


#endif
