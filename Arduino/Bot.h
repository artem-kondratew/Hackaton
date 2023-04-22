
#ifndef Servo_h
#define Servo_h


void moveForward(uint8_t speed) {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, speed);
}


void moveBackward(uint8_t speed) {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, speed);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, speed);
}


void stop() {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}


void turnRight() {
    digitalWrite(M1_DIR, BACKWARD);
    analogWrite(M1_PWM, DEFAULT_SPEED);
    digitalWrite(M2_DIR, FORWARD);
    analogWrite(M2_PWM, DEFAULT_SPEED);
}


void turnLeft() {
    digitalWrite(M1_DIR, FORWARD);
    analogWrite(M1_PWM, DEFAULT_SPEED);
    digitalWrite(M2_DIR, BACKWARD);
    analogWrite(M2_PWM, DEFAULT_SPEED);
}


#endif