
#ifndef Connection_h
#define Connection_h


#include "Config.h"
#include "Bot.h"


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
    
    message[MESSAGE_ID_CELL] = 9;

    message[MESSAGE_GOAL1_CELL] = 2;
    message[MESSAGE_GOAL1_CELL] = 3;

    message[MESSAGE_ANGLE1_CELL] = 4;
    message[MESSAGE_ANGLE2_CELL] = 5;

    message[MESSAGE_SPEED1_CELL] = 6;
    message[MESSAGE_SPEED2_CELL] = 7;

    message[MESSAGE_TORQUE1_CELL] = 8;
    message[MESSAGE_TORQUE2_CELL] = 9;

    message[MESSAGE_IS_MOVING_CELL] = 10;

    message[MESSAGE_X1_CELL] = 11;
    message[MESSAGE_X2_CELL] = 12;
    message[MESSAGE_X_SIGN] = 13;

    message[MESSAGE_Y1_CELL] = 14;
    message[MESSAGE_Y2_CELL] = 15;
    message[MESSAGE_Y_SIGN] = 16;

    message[MESSAGE_Z1_CELL] = 17;
    message[MESSAGE_Z2_CELL] = 18;
    message[MESSAGE_Z_SIGN] = 19;

    message[MESSAGE_Q01_CELL] = 20;
    message[MESSAGE_Q02_CELL] = 21;
    message[MESSAGE_Q11_CELL] = 22;
    message[MESSAGE_Q12_CELL] = 23;
    message[MESSAGE_Q21_CELL] = 24;
    message[MESSAGE_Q22_CELL] = 25;

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
            for (int cell = COMMAND_ID_CELL; cell < COMMAND_SIZE; cell++) {
                command[cell] = Serial.read();
            }
            if (!calcCommandCheckSum()) {
                findCommand();
                sendMessage();
            }
        }
    }
}


void Connection::findCommand() {
    uint16_t value = command[COMMAND_VALUE1_CELL] * 100 + command[COMMAND_VALUE2_CELL];
    if (command[COMMAND_TASK_CELL] == MOVE_FORWARD_TASK) {
        return moveForward(value);
    }
    if (command[COMMAND_TASK_CELL] == MOVE_BACKWARD_TASK) {
        return moveBackward(value);
    }
    if (command[COMMAND_TASK_CELL] == STOP_TASK) {
        return stop();
    }
    if (command[COMMAND_TASK_CELL] == TURN_RIGHT_TASK) {
        return turnRight(value);
    }
    if (command[COMMAND_TASK_CELL] == TURN_LEFT_TASK) {
        return turnLeft(value);
    }
    
}


#endif
