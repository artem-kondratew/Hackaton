
#ifndef Connection_h
#define Connection_h


#include "Bot.h"
#include "Camera.h"
#include "Claw.h"
#include "Config.h"


uint8_t command[COMMAND_SIZE];
uint8_t message[MESSAGE_SIZE];


class Connection {
private:
    static uint8_t crc8(uint8_t data[], int size);
    static uint16_t calcCommandCheckSum();
    static uint16_t calcMessageCheckSum();
    static void setMsgValues();
    
public:
static void sendMessage();
    static void receiveCommand();
    
private:
    static void findCommand();
};


uint8_t Connection::crc8(uint8_t data[], int size) {
    uint8_t byte;
    uint8_t POLY = 0x7;
    uint8_t crc8 = 0xFF;

    for (int j = 0; j < size; j++) {

        byte = data[j];
        crc8 = crc8 ^ byte;

        for (int i = 0; i < 8; i++) {

            if (crc8 & 0x80) {
                crc8 = (crc8 << 1) ^ POLY;
            }
            else {
                crc8 = crc8 << 1;
            }
        }
    }
    return crc8;
}


uint16_t Connection::calcCommandCheckSum() {
    return crc8(command, COMMAND_SIZE);
}


uint16_t Connection::calcMessageCheckSum() {
    return crc8(message, MESSAGE_SIZE-1);
}


void Connection::setMsgValues() {
    
    message[MESSAGE_START_BYTE1_CELL] = START_BYTE;
    message[MESSAGE_START_BYTE2_CELL] = START_BYTE;
    
    message[MESSAGE_ANSWER_CELL] = 1;

    message[MESSAGE_CHECKSUM_CELL] = calcMessageCheckSum();
}


void Connection::sendMessage() {
    setMsgValues();

    for (int cell = MESSAGE_START_BYTE1_CELL; cell < MESSAGE_SIZE; cell++) {
        Serial.print(char(message[cell]));
    }
}


void Connection::receiveCommand() {
    if (Serial.available() >= 7) {
        command[COMMAND_START_BYTE1_CELL] = Serial.read();
        command[COMMAND_START_BYTE2_CELL] = Serial.read();
        if (command[COMMAND_START_BYTE1_CELL] == START_BYTE && command[COMMAND_START_BYTE2_CELL] == START_BYTE) {
            for (int cell = COMMAND_TASK1_CELL; cell < COMMAND_SIZE; cell++) {
                command[cell] = Serial.read();
            }
            if (!calcCommandCheckSum()) {
                sendMessage();
                findCommand();
            }
        }
    }
}


void Connection::findCommand() {
    uint16_t value = command[COMMAND_VALUE1_CELL] * 100 + command[COMMAND_VALUE2_CELL];
    uint8_t task = command[COMMAND_TASK1_CELL] * 10 + command[COMMAND_TASK2_CELL];
    if (task == MOVE_FORWARD_TASK) {
        return Bot::moveForward(value);
    }
    if (task == MOVE_BACKWARD_TASK) {
        return Bot::moveBackward(value);
    }
    if (task == STOP_TASK) {
        return Bot::stop();
    }
    if (task == TURN_RIGHT_TASK) {
        return Bot::turnRight();
    }
    if (task == TURN_LEFT_TASK) {
        return Bot::turnLeft();
    }
    if (task == PITCH_CAMERA_TASK) {
        return Camera::pitch(value);
    }
    if (task == YAW_CAMERA_TASK) {
        return Camera::yaw(value);
    }
    if (task == CLAW_PUSH_TASK) {
        return Claw::push(value);
    }
    if (task == CLAW_POP_TASK) {
        return Claw::pop();
    }
    if (task == CLAW_ROTATE_TASK) {
        return Claw::rotateClaw(value);
    }
    if (task == CLAW_DROP_TASK) {
        return Claw::drop();
    }
    if (task == CLAW_RISE_TASK) {
        return Claw::rise();
    }
    if (task == BEEP_TASK) {
        return Bot::beep();
    }
    if (task == SHAKE_TASK) {
        return Claw::shake();
    }
    if (task == BLINK_TASK) {
        return Bot::blink(value);
    }
    if (task == SET_SPEED_TASK) {
        return Bot::set_speed(value);
    }
}


#endif
