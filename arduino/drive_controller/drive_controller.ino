#include "camera.h"
#include "config.h"
#include "gripper.h"
#include "line.h"
#include "motor.h"
#include "serial.h"


uint64_t start_time;


void robotCallback(uint8_t* msg) {
    uint8_t camera_yaw = 100;
    uint8_t camera_pitch = 100;
    uint8_t gripper_yaw = 100;
    uint8_t gripper_pitch = 0;
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

    if (task == 1) {
      LineFollower::FOLLOW_FLAG = true;
      LINE_FLAG = true;
    }
    if (task == 2) {
      LineFollower::FOLLOW_FLAG = false;
      LINE_FLAG = false;  
    }
    if (task == 3) {
      LineFollower::turn_left();
      tone(A0, 3000);
      delay(1000);
      noTone(A0);
    }
    if (task == 9) {
      Camera::set_angles(Camera::default_yaw_, Camera::default_pitch_);
      Gripper::set_angles(Gripper::default_grab_, Gripper::default_pitch_);
    }
}


void serial::set_callbacks() {
    cb = robotCallback;
}


void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    serial::init();

    serial::connect();

    Camera::init();
    Gripper::init();

    start_time = millis();

    tone(A0, 3000);
    delay(500);
    noTone(A0);
    LineFollower::calib();
    tone(A0, 3000);
    delay(500);
    noTone(A0);
}


void loop() {
    serial::receive();

    Motor::spinMotors();

    if (millis() - start_time >= 100) {
        serial::send_data();
        start_time = millis();
    }

    LineFollower::follow();
}
