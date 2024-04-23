#ifndef DRIVE_CONTROLLER_CONFIG_H
#define DRIVE_CONTROLLER_CONFIG_H


#define M0_DIR                7
#define M0_PWM                6
#define M1_DIR                4
#define M1_PWM                5

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64

#define START_BYTE0_IDX       0
#define START_BYTE1_IDX       1
#define DATA_IDX              2

#define CMD_SIZE             24
#define MSG_SIZE             35

#define POSE0_IDX             2
#define POSE1_IDX            10
#define VEL0_IDX             18
#define VEL1_IDX             22
#define TARGET0_IDX          26
#define TARGET1_IDX          30

#define CMD_VEL0_IDX          2
#define CMD_VEL1_IDX          6
#define CMD_KP_IDX           10
#define CMD_KI_IDX           14
#define CMD_KD_IDX           18
#define CMD_RESET_IDX        22


#endif // DRIVE_CONTROLLER_CONFIG_H
