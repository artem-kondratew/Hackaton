
#ifndef Bot_h
#define Bot_h


#include "Config.h"


uint8_t speed = 0;


class Bot {
public:
    static void init();
    static void set_speed(uint8_t _speed);
    static void moveForward(uint8_t speed);
    static void moveBackward(uint8_t speed);
    static void stop();
    static void turnRight();
    static void turnLeft();
    static void beep();
    static void blink(int pin);
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


void Bot::beep() {
  for(int i = 0; i < 3; i++)
  {
    tone(A1, 3000);
    delay(500);
    noTone(A1);
    delay(500);
  }
}


void Bot::blink(int pin) {
    for (int i = 0; i < 3; i++) {
        digitalWrite(pin, HIGH);
        delay(1000);
        digitalWrite(pin, LOW);
        delay(1000);
    }
}


#endif
