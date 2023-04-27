#include <chrono>
#include "graphics.h"
#include "Beep.h"


int main() {
    if (!Connect::setConnection()) {
        finish();
    }

    init_graphics();
    Beep::open();

    auto start_timer = std::chrono::system_clock::now();
    while (true) {

        key_proc(getch());

        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(TIMER)) {
            Beep::check();
            Connect::sendCommand();
            Connect::receiveMessage();
            start_timer = std::chrono::system_clock::now();
        }
    }
}
