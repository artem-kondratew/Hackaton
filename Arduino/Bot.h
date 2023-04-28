
#ifndef Bot_h
#define Bot_h


#include "Config.h"


class Bot {
public:
    static void init();
    static void moveForward(uint8_t speed);
    static void moveBackward(uint8_t speed);
    static void stop();
    static void turnRight();
    static void turnLeft();
    static void beep();
};


void Bot::init() {
    pinMode(M1_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
}


void Bot::moveForward(uint8_t speed) {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, 150);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, 150);
}


void Bot::moveBackward(uint8_t speed) {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, 150);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, 150);
}


void Bot::stop() {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}


void Bot::turnRight() {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, DEFAULT_SPEED);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, DEFAULT_SPEED);
}


void Bot::turnLeft() {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, DEFAULT_SPEED);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, DEFAULT_SPEED);
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





#endif
