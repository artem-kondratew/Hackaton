
#ifndef Claw_h
#define Claw_h


#include <Servo.h>
#include "Config.h"


Servo claw_servo;    // Claw       D8
Servo rotate_servo;  // Claw mover D9


class Claw {  
public:
    static void rotateClaw(uint8_t angle);
    static void push(uint8_t angle);
    static void pop();
    static void setStartPosition();
    static void init();
    static void drop();
    static void rise();
    static void shake();
};


void Claw::rotateClaw(uint8_t angle){
    if(angle > ROTATE_MAX_ANGLE) {
        angle = ROTATE_MAX_ANGLE;
    }
    if(angle < ROTATE_MIN_ANGLE) {
        angle = ROTATE_MIN_ANGLE;
    }
    rotate_servo.write(angle);
}

void Claw::push(uint8_t angle){
    if(angle > CLAW_MAX_ANGLE) {
        angle = CLAW_MAX_ANGLE;
    }
    if(angle < CLAW_MIN_ANGLE) {
        angle = CLAW_MIN_ANGLE;
    }
    claw_servo.write(angle);
}


void Claw::pop() {
    claw_servo.write(CLAW_MAX_ANGLE);
}


void Claw::setStartPosition(){
    rotateClaw(ROTATE_DEFAULT_ANGLE);
    push(CLAW_DEFAULT_ANGLE);
}
    
    
void Claw::init(){
    rotate_servo.attach(9);
    claw_servo.attach(8);
    setStartPosition();
}


void Claw::drop() {
    rotateClaw(ROTATE_MIN_ANGLE);
}


void Claw::rise() {
    rotateClaw(ROTATE_MAX_ANGLE);
}


void Claw::shake() {
    drop();
    push(0);
    rotateClaw(30);
    delay(300);
    rotateClaw(20);
    delay(300);
    rotateClaw(30);
    delay(300);
    rotateClaw(20);
    delay(300);
    rotateClaw(30);
    delay(300);
    rotateClaw(20);
    drop();
}


#endif
