
#include "Bot.h"
#include "Camera.h"
#include "Claw.h"
#include "Connection.h"
#include "Config.h"


void yield() {
    Connection::receiveCommand();
}


void setup() {
    pinMode(13, OUTPUT);
    pinMode(7, OUTPUT);
    digitalWrite(7,0);
    
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Bot::init();
    Camera::init();
    Claw::init();
}


unsigned long end_timer = 0;
unsigned long start_timer = 0;


void loop() {
    Connection::receiveCommand();
}
