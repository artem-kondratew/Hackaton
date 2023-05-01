
#ifndef Bot_h
#define Bot_h


#include "Config.h"


uint8_t speed = 0;

uint8_t beep_count;
bool beep_flag = false;

uint8_t blink_count;
bool blink_flag = false;


class Bot {
public:
    static void init();
    static void set_speed(uint8_t _speed);
    static void moveForward(uint8_t speed);
    static void moveBackward(uint8_t speed);
    static void stop();
    static void turnRight();
    static void turnLeft();
    static void beep(bool start=false);
    static void blink(bool start=false);
};


void Bot::init() {
    speed = DEFAULT_SPEED;
    pinMode(M1_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
}


void Bot::set_speed(uint8_t _speed) {
    speed = _speed;
}


void Bot::moveForward(uint8_t speed) {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::moveBackward(uint8_t speed) {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, speed);
}


void Bot::stop() {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}


void Bot::turnRight() {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, ROTATE_SPEED);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, ROTATE_SPEED);
}


void Bot::turnLeft() {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, ROTATE_SPEED);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, ROTATE_SPEED);
}


void Bot::beep(bool start=false) {
    if (beep_count == 0) {
        if (start) {
            beep_count = 5;
            beep_flag = !beep_flag;
            tone(A1, 3000);
        }
        return;
    }
    beep_flag = !beep_flag;
    if (beep_flag) {
        tone(A1, 3000);
    }
    else {
        noTone(A1);
    }
    beep_count--;
}


void Bot::blink(bool start=false) {
    if (blink_count == 0) {
        if (start) {
            blink_count = 5;
            blink_flag = !blink_flag;
            digitalWrite(LED_BUILTIN, blink_flag);
        }
        return;
    }
    blink_flag = !blink_flag;
    digitalWrite(LED_BUILTIN, blink_flag);
    blink_count--;
}


#endif
