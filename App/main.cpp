#include "header.h"
#include "graphics.h"


int main() {
    if (!Connect::setConnection()) {
        finish();
    }

    init_graphics();

    std::thread graphics_thr(key_proc);
    std::thread connect_thr(Connect::exchange);
    std::thread sighandler_thr(signal, SIGINT, sighandler);
    std::thread vision_thr(Vision::processing);

    graphics_thr.join();
    connect_thr.join();
    sighandler_thr.join();
    vision_thr.join();

    finish();
}
