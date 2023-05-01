
#include "Bot.h"
#include "Camera.h"
#include "Claw.h"
#include "Connection.h"
#include "Config.h"


unsigned long long int beep_timer;
unsigned long long int blink_timer;


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Bot::init();
    Camera::init();
    Claw::init();

    beep_timer = millis();
    blink_timer = millis();
}


void loop() {
    Connection::receiveCommand();
    if (millis() - beep_timer > BLINK_TIMER) {
        Bot::beep();
        beep_timer = millis();
    }
    if (millis() - blink_timer > BLINK_TIMER) {
        Bot::blink();
        blink_timer = millis();
    }
}
