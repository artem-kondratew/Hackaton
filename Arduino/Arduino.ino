
#include "Bot.h"
#include "Claw.h"
#include "Connection.h"
#include "Config.h"


void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    pinMode(M1_DIR, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    //turnRight(180);
}


void loop() {
    Connection::receiveCommand();
    
}
