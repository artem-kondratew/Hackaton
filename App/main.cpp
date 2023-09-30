#include <chrono>
#include "graphics.h"


int main() {
    if (!Connect::setConnection()) {
        finish();
    }

    init_graphics();

    auto start_timer = std::chrono::system_clock::now();
    while (true) {

        key_proc(getch());

        auto end_timer = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count() > int(TIMER)) {
            Connect::sendCommand();
            Connect::receiveMessage();
            start_timer = std::chrono::system_clock::now();
        }
    }
}
