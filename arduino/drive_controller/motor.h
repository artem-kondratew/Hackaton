#ifndef DRIVE_CONTROLLER_MOTOR_H
#define DRIVE_CONTROLLER_MOTOR_H


const uint8_t MOTORS = 2;

int64_t msg_poses[MOTORS];
float msg_vels[MOTORS];
float msg_targets[MOTORS];

float cmd_vels[MOTORS];

bool state = false;

float kp = 1;
float ki = 0;
float kd = 0;


class Motor {
private:
    uint8_t dir_;
    uint8_t pwm_;
    float cmd_vel_;

public:
    Motor(uint8_t dir, uint8_t pwm);
    ~Motor() = default;

public:
    void setPwm(uint8_t dir, uint8_t pwm);
    
    void spin();
    static void spinMotors();

    void set_cmd_vel(float cmd_vel);
    
    static void setVelocities(float* vels);

    static void callback(uint8_t* msg);
};


Motor motor0(M0_DIR, M0_PWM);
Motor motor1(M1_DIR, M1_PWM);


Motor::Motor(uint8_t dir, uint8_t pwm) {
    dir_ = dir;
    pwm_ = pwm;

    pinMode(dir_, OUTPUT);
    pinMode(pwm_, OUTPUT);
}


void Motor::setPwm(uint8_t dir, uint8_t pwm) {
    digitalWrite(dir_, dir);
    analogWrite(pwm_, pwm);
}


void Motor::spin() {
    uint8_t pwm = abs(cmd_vel_);
    int dir = cmd_vel_ < 0 ? 0 : 1;
    setPwm(dir, pwm);
}


void Motor::spinMotors() {
    motor0.spin();
    motor1.spin();
}


void Motor::set_cmd_vel(float cmd_vel) {
    cmd_vel_ = cmd_vel > 255 ? 255 : cmd_vel;
    cmd_vel_ = cmd_vel_ < -255 ? -255 : cmd_vel_;
}


void Motor::callback(uint8_t* msg) {
    uint8_t reset = 0;
    memcpy(&reset, msg + CMD_RESET_IDX, sizeof(uint8_t));
    
    memcpy(cmd_vels + 0, msg + CMD_VEL0_IDX, sizeof(float));
    memcpy(cmd_vels + 1, msg + CMD_VEL1_IDX, sizeof(float));

    motor0.set_cmd_vel(cmd_vels[0]);
    motor1.set_cmd_vel(cmd_vels[1]);
}


#endif // DRIVE_CONTROLLER_MOTOR_H
