
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
}


void loop() {
    Connection::receiveCommand();
}
