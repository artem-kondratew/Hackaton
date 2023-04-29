
#ifndef Camera_h
#define Camera_h


#include <Servo.h>


#define PITCH_MIN_ANGLE      40
#define PITCH_MAX_ANGLE     180
#define YAW_MIN_ANGLE         0
#define YAW_MAX_ANGLE       180

#define PITCH_DEFAULT_ANGLE 100
#define YAW_DEFAULT_ANGLE   100


Servo pitch_servo;  // Bottom servo D10
Servo yaw_servo;    // Head servo   D11


class Camera{
public:
    static void setStartPosition();
    static void init();
    static void pitch(uint8_t angle);
    static void yaw(uint8_t angle);
};


void Camera::setStartPosition() {
    yaw_servo.write(YAW_DEFAULT_ANGLE);
    pitch_servo.write(PITCH_DEFAULT_ANGLE);
}


void Camera::init() {
    yaw_servo.attach(11);
    pitch_servo.attach(10);
    setStartPosition();
}


void Camera::pitch(uint8_t angle) {
    if(angle > PITCH_MAX_ANGLE) {
        angle = PITCH_MAX_ANGLE;
    }
    if(angle < PITCH_MIN_ANGLE) {
        angle = PITCH_MIN_ANGLE;
    }
    pitch_servo.write(angle);
}


void Camera::yaw(uint8_t angle){
    if(angle > YAW_MAX_ANGLE) {
        angle = YAW_MAX_ANGLE;
    }
    if(angle < YAW_MIN_ANGLE) {
        angle = YAW_MIN_ANGLE;
    }
    yaw_servo.write(angle);
}


#endif
