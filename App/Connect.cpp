//
// Created by user on 05.03.23.
//

#include "Connect.h"


void Connect::exchange() {
    auto start_timer = std::chrono::system_clock::now();
    while (true) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > TIMER) {
            Connect::sendCommand();
            Connect::receiveMessage();
            start_timer = std::chrono::system_clock::now();
        }
    }
}


void Connect::resetCommand() {
    command[COMMAND_START_BYTE1_CELL] = START_BYTE;
    command[COMMAND_START_BYTE2_CELL] = START_BYTE;
    command[COMMAND_TASK1_CELL] = 0;
    command[COMMAND_TASK2_CELL] = PING_TASK;
    command[COMMAND_VALUE1_CELL] = PING_VALUE1;
    command[COMMAND_VALUE2_CELL] = PING_VALUE2;
    calcCommandCheckSum();
}


bool Connect::openArduino() {
    if (Arduino == -1) {
        Arduino = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (Arduino == -1) {
            return false;
        }
    }

    tcgetattr(Arduino, &SerialPortSettings);

    SerialPortSettings.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;     // 8 bit chars
    SerialPortSettings.c_cflag &= ~(PARENB | PARODD);  // shut off parody
    SerialPortSettings.c_cflag &= ~CSTOPB; //no scts stop
    SerialPortSettings.c_iflag &= ~IGNBRK; //disable break processing
    SerialPortSettings.c_iflag = 0;        // no echo
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
    SerialPortSettings.c_oflag = 0;        // no remapping
    SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG |IEXTEN);
    SerialPortSettings.c_cc[VMIN] = 0;     // read doesn't block
    SerialPortSettings.c_cc[VTIME] = 0;    // 0s read timeout

    tcsetattr(Arduino,TCSANOW,&SerialPortSettings);

    return true;
}


bool Connect::setConnection() {
    if (!openArduino()) {
        std::cout << "Unable to connect" << std::endl;
        return false;
    }

    resetCommand();
    bool message_flag = false;
    auto start_timer = std::chrono::system_clock::now();
    while (!message_flag) {
        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(TIMER)) {
            sendCommand();
            message_flag = receiveMessage();
            start_timer = std::chrono::system_clock::now();
        }
    }
    std::cout << "connected" << std::endl;
    initImportantCommandMap();
    initCommandMap();
    return true;
}


void Connect::disconnectArduino() {
    close(Arduino);
}


uint8_t Connect::crc8(const uint8_t pocket[], uint64_t size) {
    uint8_t BYTE_SIZE = 8;
    uint8_t MSB_MASK = 0x80;
    uint8_t byte;
    uint8_t POLY = 0x7;
    uint8_t crc8 = 0xFF;

    for (int cell = 0; cell < size; cell++) {

        byte = pocket[cell];
        crc8 = crc8 ^ byte;

        for (int byte_number = 0; byte_number < BYTE_SIZE; byte_number++) {

            if (crc8 & MSB_MASK) {
                crc8 = (crc8 << 1) ^ POLY;
            }
            else {
                crc8 = crc8 << 1;
            }
        }
    }
    return crc8;
}


void Connect::calcCommandCheckSum() {
    command[COMMAND_CHECKSUM_CELL] = crc8(command, COMMAND_SIZE - 1);
}


uint8_t Connect::calcMessageCheckSum(uint8_t buffer[]) {
    return crc8(buffer, MESSAGE_SIZE);
}


void Connect::sendCommand() {
    if (!openArduino()) {
        return;
    }

    calcCommandCheckSum();
    write(Arduino, command, COMMAND_SIZE);
    resetCommand();
    connect_mutex.unlock();
}


void Connect::setTask(uint8_t task) {
    command[COMMAND_TASK1_CELL] = task / 10;
    command[COMMAND_TASK2_CELL] = task % 10;
}


void Connect::setValue(uint16_t value) {
    command[COMMAND_VALUE1_CELL] = uint8_t(value / 100);
    command[COMMAND_VALUE2_CELL] = uint8_t(value % 100);
}


void Connect::encodeCommand(uint64_t cmd) {
    auto task = static_cast<uint8_t>(cmd / 10000);
    setTask(task);
    uint16_t value = cmd % 10000;
    setValue(value);
}


Gservo* Connect::findGservo(uint8_t id) {
    if (id == 1) {
        return &gservo1;
    }
    if (id == 2) {
        return &gservo2;
    }
    if (id == 3) {
        return &gservo3;
    }
    if (id == 4) {
        return &gservo4;
    }
}


bool Connect::receiveMessage() {
    if (!openArduino()) {
        return false;
    }

    uint8_t buf[MESSAGE_SIZE];
    read(Arduino, buf, MESSAGE_SIZE);

    if (buf[MESSAGE_START_BYTE1_CELL] == START_BYTE && buf[MESSAGE_START_BYTE2_CELL] == START_BYTE) {
        if (!calcMessageCheckSum(buf)) {
            std::memcpy(message, buf, sizeof(uint8_t) * MESSAGE_SIZE);
            //Connect::decodeMessage();
            memset(buf,0,MESSAGE_SIZE);
            return true;
        }
    }
    return false;
}


uint64_t Connect::checkNumberCommand(std::string s) {
    uint8_t numbers[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint64_t flag = 0;
    for (int i = 0; i < key_cmd.size(); i++) {
        for (uint8_t number: numbers) {
            if (s[i] == number) {
                flag++;
                break;
            }
        }
    }
    return flag;
}


void Connect::stop() {
    connect_mutex.lock();
    resetCommand();
    setTask(STOP_TASK);
}


void Connect::moveForward() {
    connect_mutex.lock();
    resetCommand();
    setTask(MOVE_FORWARD_TASK);
}


void Connect::moveBackward() {
    connect_mutex.lock();
    resetCommand();
    setTask(MOVE_BACKWARD_TASK);
}


void Connect::turnRight() {
    connect_mutex.lock();
    resetCommand();
    setTask(TURN_RIGHT_TASK);
}


void Connect::turnLeft() {
    connect_mutex.lock();
    resetCommand();
    setTask(TURN_LEFT_TASK);
}


void Connect::push() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_PUSH_TASK);
    setValue(0);
}


void Connect::pop() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_POP_TASK);
}


void Connect::rise() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_RISE_TASK);
}


void Connect::drop() {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_DROP_TASK);
}


void Connect::beep() {
    connect_mutex.lock();
    resetCommand();
    setTask(BEEP_TASK);
}


void Connect::rotate(uint8_t angle) {
    connect_mutex.lock();
    resetCommand();
    setTask(CLAW_ROTATE_TASK);
    setValue(angle);
}


void Connect::shake() {
    connect_mutex.lock();
    resetCommand();
    setTask(SHAKE_TASK);
}


void Connect::blink() {
    connect_mutex.lock();
    resetCommand();
    setTask(BLINK_TASK);
}


void Connect::decodeKeyInput() {

    if (checkNumberCommand(key_cmd.get_str()) == key_cmd.size()) {
        Connect::encodeCommand(stoi(key_cmd.get_str()));
        return;
    }

    if (imp_command_map.count(key_cmd.get_str())) {
        return imp_command_map[key_cmd.get_str()]();
    }
    if (Vision::is_processing()) {
        return;
    }

    if (command_map.count(key_cmd.get_str())) {
        return command_map[key_cmd.get_str()]();
    }

    if (key_cmd.get_str().substr(0, 4) == "rot ") {
        if (checkNumberCommand(key_cmd.get_str().substr(4, 3))) {
            return rotate(stoi(key_cmd.get_str().substr(4, 3)));
        }
    }
}


void Connect::initImportantCommandMap() {
    imp_command_map["start vision"] = Vision::start_processing;
    imp_command_map["stop vision"] = Vision::stop_processing;
    imp_command_map["beep"] = Connect::beep;
    imp_command_map["blink"] = Connect::blink;
}


void Connect::initCommandMap() {
    command_map["stop"] = stop;
    command_map["push"] = push;
    command_map["pop"] = pop;
    command_map["rise"] = rise;
    command_map["drop"] = drop;
    command_map["shake"] = shake;
}
