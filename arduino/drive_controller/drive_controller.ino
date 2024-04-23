#include "serial.h"
#include "camera.h"
#include "config.h"
#include "gripper.h"
#include "motor.h"


uint64_t start_time;


void robotCallback(uint8_t* msg) {
    uint8_t camera_yaw;
    uint8_t camera_pitch;
    uint8_t gripper_yaw;
    uint8_t gripper_pitch;
    uint8_t task = 0;
    
    memcpy(cmd_vels + 0, msg + CMD_VEL0_IDX, sizeof(float));
    memcpy(cmd_vels + 1, msg + CMD_VEL1_IDX, sizeof(float));
    memcpy(&camera_yaw, msg + CMD_CAMERA_YAW_IDX, sizeof(uint8_t));
    memcpy(&camera_pitch, msg + CMD_CAMERA_PITCH_IDX, sizeof(uint8_t));
    memcpy(&gripper_yaw, msg + CMD_GRIPPER_YAW_IDX, sizeof(uint8_t));
    memcpy(&gripper_pitch, msg + CMD_GRIPPER_PITCH_IDX, sizeof(uint8_t));
    memcpy(&task, msg + CMD_TASK_IDX, sizeof(uint8_t));

    motor0.set_cmd_vel(cmd_vels[0]);
    motor1.set_cmd_vel(cmd_vels[1]);

    Camera::set_angles(camera_yaw, camera_pitch);
    Gripper::set_angles(gripper_yaw, gripper_pitch);
}


void serial::set_callbacks() {
    cb = robotCallback;
}


void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    //Camera::init();
    Gripper::init();

    serial::init();

    serial::connect();

    start_time = millis();
}


void loop() {
    serial::receive();
    
    Motor::spinMotors();
    Camera::spin();
    Gripper::spin();

    if (millis() - start_time >= 100) {
        serial::send_data();
        start_time = millis();
    }
}
