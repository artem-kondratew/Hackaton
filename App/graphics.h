//
// Created by user on 05.03.23.
//

#ifndef MANIPULATOR_GRAPHICS_H
#define MANIPULATOR_GRAPHICS_H


#include <csignal>
#include <sys/ioctl.h>
#include "ncurses.h"
#include "Connect.h"
#include "History.h"


#define KEY_RETURN     10

#define ID_X            9
#define GOAL_X         17
#define ANGLE_X        23
#define SPEED_X        30
#define TORQUE_X       37
#define IS_MOVING_X    45
#define EDGE_X         55
#define X_X            57
#define Y_X            62
#define Z_X            67
#define Q0_X           57
#define Q1_X           62
#define Q2_X           67
#define ALL_Q_TITLE_Y   4
#define ALL_Q_Y         5

#define COMMAND_Y       8
#define LAST_COMMAND_Y  9


int CURS_Y = 0;
int CURS_X = 0;


int get_columns() {
    struct winsize window{};
    ioctl(0, TIOCGWINSZ, &window);
    return window.ws_col;
}


int get_rows() {
    struct winsize window{};
    ioctl(0, TIOCGWINSZ, &window);
    return window.ws_row;
}


void finish() {
    Connect::disconnectArduino();
    resetty();
    endwin();
    exit(0);
}


void sighandler(int sig) {
    if (sig == SIGINT) {
        finish();
    }
}


void print_table() {
    for (int i = 1; i < 5; i++) {
        move(i + 1, 0);
        printw("servo%d", i);
    }

    move(0, ID_X);
    printw("id");

    move(0, GOAL_X);
    printw("goal");
    move(0, ANGLE_X);
    printw("angle");

    move(0, SPEED_X);
    printw("speed");

    move(0, TORQUE_X);
    printw("torque");

    move(0, IS_MOVING_X);
    printw("is_moving");

    move(0, X_X);
    printw("x");

    move(0, Y_X);
    printw("y");

    move(0, Z_X);
    printw("z");

    move(COMMAND_Y - 1, 0);
    printw("Set command:");

    move(ALL_Q_TITLE_Y, Q0_X);
    printw("q0");

    move(ALL_Q_TITLE_Y, Q1_X);
    printw("q1");

    move(ALL_Q_TITLE_Y, Q2_X);
    printw("q2");

    for (int i = 0; i < 7; i++) {
        move(i, EDGE_X);
        printw("|");
    }

    refresh();
}


void print_id() {
    for (int i = 1; i < 5; i++) {
        move(i + 1, ID_X + 1);
        printw("%d", i);
    }
    refresh();
}


void print_exit_option() {
    move(get_rows() - 1, 0);
    printw("Press 'Ctrl+C' to exit");
    refresh();
}


void init_graphics() {
    initscr();
    savetty();  //  save terminal settings

    //nonl();  //  deny going to the new line

    cbreak();  //  send buffer after pressing enter
    echo();  //  visible printing
    timeout(0);

    //leaveok(stdscr, TRUE);
    //curs_set(0);  //  hide cursor
    keypad(stdscr, TRUE);

    signal(SIGINT, sighandler);

    clear();
    print_table();
    print_id();
    print_exit_option();
    move(COMMAND_Y, 0);
    refresh();
}


void clear_command_line() {
    move(COMMAND_Y, 0);
    for (int i = 0; i < get_columns(); i++) {
        printw(" ");
    }
    move(COMMAND_Y, 0);
    refresh();
}


void print_last_command() {
    move(LAST_COMMAND_Y, 0);
    for (int i = 0; i < get_columns(); i++) {
        printw(" ");
    }
    move(LAST_COMMAND_Y, 0);
    printw("%s", Connect::key_cmd.get());
}


void print_command_line() {
    move(COMMAND_Y, 0);
    printw("%s", Connect::key_cmd.get());
}


void key_return_proc() {
    if (Connect::key_cmd.get_str().empty()) {
        return;
    }
    History::append(Connect::key_cmd);
    Connect::decodeKeyInput();
    print_last_command();
    clear_command_line();
    Connect::key_cmd.reset();
    History::resetIndex();
}


void key_backspace_proc() {
    getsyx(CURS_Y, CURS_X);
    Connect::key_cmd.setCurs(CURS_X + 1);
    Connect::key_cmd.keyBackspace();
    clear_command_line();
    print_command_line();
    move(CURS_Y, CURS_X);
}


void key_delete_proc() {
    getsyx(CURS_Y, CURS_X);
    Connect::key_cmd.setCurs(CURS_X);
    Connect::key_cmd.keyDelete();
    clear_command_line();
    print_command_line();
    move(CURS_Y, CURS_X);
}


void key_left_proc() {
    //getsyx(CURS_Y, CURS_X);
    //move(CURS_Y, CURS_X - 1);
    Connect::resetCommand();
    Connect::turnLeft();
}


void key_right_proc() {
    /*getsyx(CURS_Y, CURS_X);
    if (CURS_X == Connect::key_cmd.size()) {
        return;
    }
    move(CURS_Y, CURS_X + 1);*/
    Connect::resetCommand();
    Connect::turnRight();
}


void key_up_proc() {
    /*if (History::getIndex() == 0) {
        History::setCurrentCommand(Connect::key_cmd.get());
    }
    History::moveUp();
    Connect::key_cmd.set(History::get());
    clear_command_line();
    print_command_line();*/
    Connect::resetCommand();
    Connect::moveForward();
}


void key_down_proc() {
    /*History::moveDown();
    Connect::key_cmd.set(History::get());
    clear_command_line();
    print_command_line();*/
    Connect::resetCommand();
    Connect::moveBackward();
}


void key_proc(int key) {
    auto symbol = static_cast<uint8_t>(key);
    /*if (Connect::manipulate_flag) {
        manipulate_proc();
    }*/

    if (key == KEY_RETURN) {
        return key_return_proc();
    }
    if (key == KEY_BACKSPACE) {
        return key_backspace_proc();
    }
    if (key == KEY_DC) {
        return key_delete_proc();
    }
    if (key == KEY_LEFT) {
        return key_left_proc();
    }
    if (key == KEY_RIGHT) {
        return key_right_proc();
    }
    if (key == KEY_UP) {
        return key_up_proc();
    }
    if (key == KEY_DOWN) {
        return key_down_proc();
    }
    if (key == ERR) {
        return;
    }

    getsyx(CURS_Y, CURS_X);
    Connect::key_cmd.push(symbol, CURS_X - 1);
    clear_command_line();
    print_command_line();
    move(CURS_Y, Connect::key_cmd.getCurs() + 1);
    refresh();
}


void print_param(uint8_t gservo_id, int x, uint16_t param, bool is_q=false) {
    int y = (is_q) ? ALL_Q_Y : 1 + gservo_id;
    move(y, x);
    printw("    ");
    move(y, x);
    printw("%d", param);
}


void print_goal(uint8_t gservo_id, uint16_t goal) {
    print_param(gservo_id, GOAL_X, goal);
}


void print_angle(uint8_t gservo_id, uint16_t angle) {
    print_param(gservo_id, ANGLE_X + 1, angle);
}


void print_speed(uint8_t gservo_id, uint16_t speed) {
    print_param(gservo_id, SPEED_X + 1, speed);
}


void print_torque(uint8_t gservo_id, uint16_t torque) {
    print_param(gservo_id, TORQUE_X + 1, torque);
}


void print_is_moving(uint8_t gservo_id, uint16_t is_moving) {
    print_param(gservo_id, IS_MOVING_X, is_moving);
}


void print_x(uint16_t x) {
    //print_param(DXL_ID1, X_X, x);
}


void print_y(uint16_t y) {
    //print_param(DXL_ID1, Y_X, y);
}


void print_z(uint16_t z) {
    //print_param(DXL_ID1, Z_X, z);
}


void print_q0(uint16_t q0) {
    print_param(ALL_Q_Y, Q0_X, q0);
}


void print_q1(uint16_t q1) {
    print_param(ALL_Q_Y, Q1_X, q1);
}


void print_q2(uint16_t q2) {
    print_param(ALL_Q_Y, Q2_X, q2);
}


void print_params_from_servo(Gservo gservo) {
    print_goal(gservo.get_id(), gservo.get_goal());
    print_angle(gservo.get_id(), gservo.get_angle());
    print_speed(gservo.get_id(), gservo.get_speed());
    print_torque(gservo.get_id(), gservo.get_torque());
    print_is_moving(gservo.get_id(), gservo.get_is_moving());
    print_x(gservo.get_x());
    print_y(gservo.get_y());
    print_z(gservo.get_z());
    print_q0(gservo.get_q0());
    print_q1(gservo.get_q1());
    print_q2(gservo.get_q2());
}


void print_params() {
    getsyx(CURS_Y, CURS_X);
    print_params_from_servo(gservo1);
    print_params_from_servo(gservo2);
    print_params_from_servo(gservo3);
    print_params_from_servo(gservo4);
    move(CURS_Y, CURS_X);
}


#endif //MANIPULATOR_GRAPHICS_H
