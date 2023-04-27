
#include "Bot.h"
#include "Camera.h"
#include "Claw.h"
#include "Connection.h"
#include "Config.h"


void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Bot::init();
    Camera::init();
    Claw::init();
    Bot::piezoBeep();
}


void loop() {
    //Connection::receiveCommand();
}                                      
