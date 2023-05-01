
#ifndef Config_h
#define Config_h


#define SERIAL_BAUDRATE       9600

#define M1_DIR                   4
#define M1_PWM                   5
#define M2_DIR                   7
#define M2_PWM                   6

#define BEEP_PIN                A1

#define LEFT                     0
#define RIGHT                    1
#define FORWARD                  1
#define BACKWARD                 0

#define DEFAULT_SPEED          120
#define ROTATE_SPEED           120

#define TIMER                  100
#define BLINK_TIMER            200
#define BEEP_TIMER             500
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
#define MESSAGE_ANSWER_CELL      2
#define MESSAGE_CHECKSUM_CELL    3
#define MESSAGE_SIZE             4

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
#define BLINK_TASK              14
#define SET_SPEED_TASK          15

#define PING_DXL_ID              0
#define PING_TASK        STOP_TASK
#define PING_VALUE1              0
#define PING_VALUE2              0

#define CLAW_MIN_ANGLE          35
#define CLAW_MAX_ANGLE         150
#define ROTATE_MIN_ANGLE         5
#define ROTATE_MAX_ANGLE       100

#define ROTATE_DEFAULT_ANGLE   100
#define CLAW_DEFAULT_ANGLE       0


#endif
