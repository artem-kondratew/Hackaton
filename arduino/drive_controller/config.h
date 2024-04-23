#ifndef DRIVE_CONTROLLER_CONFIG_H
#define DRIVE_CONTROLLER_CONFIG_H


#define CMD_SIZE              16
#define MSG_SIZE              35

#define SERIAL_BAUDRATE  2000000
#define START_BYTE            64

#define START_BYTE0_IDX        0
#define START_BYTE1_IDX        1
#define DATA_IDX               2

#define CMD_VEL0_IDX           2
#define CMD_VEL1_IDX           6
#define CMD_CAMERA_YAW_IDX    10
#define CMD_CAMERA_PITCH_IDX  11
#define CMD_GRIPPER_YAW_IDX   12
#define CMD_GRIPPER_PITCH_IDX 13
#define CMD_TASK_IDX          14


#endif // DRIVE_CONTROLLER_CONFIG_H
