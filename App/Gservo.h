//
// Created by user on 3/18/23.
//

#ifndef MANIPULATOR_GSERVO_H
#define MANIPULATOR_GSERVO_H


#include <cstdint>


class Gservo {
private:
    uint8_t id;
    uint16_t goal;
    uint16_t angle;
    uint16_t speed;
    uint16_t torque;
    uint8_t is_moving;
    inline static int x;
    inline static int y;
    inline static int z;
    inline static uint16_t q0;
    inline static uint16_t q1;
    inline static uint16_t q2;

public:
    explicit Gservo(uint8_t _id);

    uint8_t get_id() const;
    uint16_t get_goal() const;
    uint16_t get_angle() const;
    uint16_t get_speed() const;
    uint16_t get_torque() const;
    uint16_t get_is_moving() const;
    static uint16_t get_x();
    static uint16_t get_y();
    static uint16_t get_z();
    static uint16_t get_q0() ;
    static uint16_t get_q1() ;
    static uint16_t get_q2() ;

    void set_goal(uint8_t _goal1, uint8_t _goal2);
    void set_angle(uint8_t _angle1, uint8_t _angle2);
    void set_speed(uint8_t _speed1, uint8_t _speed2);
    void set_torque(uint8_t _torque1, uint8_t _torque2);
    void set_is_moving(uint8_t _is_moving);
    static void set_x(uint8_t _x1, uint8_t _x2, bool x_sign);
    static void set_y(uint8_t _y1, uint8_t _y2, bool y_sign);
    static void set_z(uint8_t _z1, uint8_t _z2, bool z_sign);
    static void set_q0(uint8_t _q01, uint8_t _q02);
    static void set_q1(uint8_t _q11, uint8_t _q12);
    static void set_q2(uint8_t _q21, uint8_t _q22);
};


inline Gservo gservo1(1);
inline Gservo gservo2(2);
inline Gservo gservo3(3);
inline Gservo gservo4(4);


#endif //MANIPULATOR_GSERVO_H
