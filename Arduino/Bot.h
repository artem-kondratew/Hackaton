
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
};


void Bot::init() {
    pinMode(M1_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
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


void PiezoBeep(int PiezoPin, int Tone, int TimeON, int TimeOFF, int Count)
{
  for(int i = 0; i < Count; i++)
  {
    tone(PiezoPin, Tone);
    delay(TimeON);
    noTone(PiezoPin);
    delay(TimeOFF);
  }
}


#endif
