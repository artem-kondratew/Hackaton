
#ifndef Config_h
#define Config_h


#define SERIAL_BAUDRATE       9600

#define M1_DIR                   4
#define M1_PWM                   5
#define M2_DIR                   7
#define M2_PWM                   6

#define LEFT                     0
#define RIGHT                    1
#define FORWARD                  1
#define BACKWARD                 0

#define DEFAULT_SPEED          120

#define TIMER                  100
#define START_BYTE              64

#define COMMAND_START_BYTE1_CELL 0
#define COMMAND_START_BYTE2_CELL 1
#define COMMAND_TASK1_CELL       2
#define COMMAND_TASK2_CELL       3
#define COMMAND_VALUE1_CELL      4
#define COMMAND_VALUE2_CELL      5
#define COMMAND_CHECKSUM_CELL    6
#define COMMAND_SIZE             7

#define MESSAGE_START_BYTE1_CELL 0
#define MESSAGE_START_BYTE2_CELL 1
#define MESSAGE_ID_CELL          2
#define MESSAGE_GOAL1_CELL       3
#define MESSAGE_GOAL2_CELL       4
#define MESSAGE_ANGLE1_CELL      5
#define MESSAGE_ANGLE2_CELL      6
#define MESSAGE_SPEED1_CELL      7
#define MESSAGE_SPEED2_CELL      8
#define MESSAGE_TORQUE1_CELL     9
#define MESSAGE_TORQUE2_CELL    10
#define MESSAGE_IS_MOVING_CELL  11
#define MESSAGE_X1_CELL         12
#define MESSAGE_X2_CELL         13
#define MESSAGE_X_SIGN          14
#define MESSAGE_Y1_CELL         15
#define MESSAGE_Y2_CELL         16
#define MESSAGE_Y_SIGN          17
#define MESSAGE_Z1_CELL         18
#define MESSAGE_Z2_CELL         19
#define MESSAGE_Z_SIGN          20
#define MESSAGE_Q01_CELL        21
#define MESSAGE_Q02_CELL        22
#define MESSAGE_Q11_CELL        23
#define MESSAGE_Q12_CELL        24
#define MESSAGE_Q21_CELL        25
#define MESSAGE_Q22_CELL        26
#define MESSAGE_CHECKSUM_CELL   27
#define MESSAGE_SIZE            28

#define MOVE_BACKWARD_TASK       0
#define MOVE_FORWARD_TASK        1
#define STOP_TASK                2
#define TURN_RIGHT_TASK          3
#define TURN_LEFT_TASK           4
#define PITCH_CAMERA_TASK        5
#define YAW_CAMERA_TASK          6
#define CLAW_PUSH_TASK           7
#define CLAW_POP_TASK            8
#define CLAW_ROTATE_TASK         9
#define CLAW_DROP_TASK          10
#define CLAW_RISE_TASK          11
#define BEEP_TASK               12
#define SHAKE_TASK              13

#define PING_DXL_ID             35
#define PING_TASK        STOP_TASK
#define PING_VALUE1             37
#define PING_VALUE2             61

#define CLAW_MIN_ANGLE        35
#define CLAW_MAX_ANGLE       150
#define ROTATE_MIN_ANGLE       5
#define ROTATE_MAX_ANGLE     100

#define ROTATE_DEFAULT_ANGLE 100
#define CLAW_DEFAULT_ANGLE     0


#endif
