/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

/* Board address: ex 0x10 */
#define BOARD_ADDRESS                   0x10

#define REG_DEF_PID                     0xdf
#define REG_DEF_VID                     0x10

#define STEPPER_COUNT                   0x01
#define MOTOR_COUNT                     0x02

#define REG_SLAVE_ADDR                  0x00
#define REG_PID                         0x01
#define REG_PVD                         0x02
#define REG_CTRL_MODE                   0x03
#define REG_ENCODER1_EN                 0x04
#define REG_ENCODER1_SPPED              0x05
#define REG_ENCODER1_REDUCTION_RATIO    0x07
#define REG_ENCODER2_EN                 0x09
#define REG_ENCODER2_SPEED              0x0a
#define REG_ENCODER2_REDUCTION_RATIO    0x0c
#define REG_MOTOR_PWM                   0x0e
#define REG_MOTOR1_ORIENTATION          0x0f
#define REG_MOTOR1_SPEED                0x10
#define REG_MOTOR2_ORIENTATION          0x12
#define REG_MOTOR2_SPEED                0x13

/* Orientation  */
#define CW                              0x01
#define CCW                             0x02
#define STOP                            0x03
#define MOTOR_ALL                       0xFFFFFFFF


typedef enum MOTOR_ID_E{
    MOTOR_1 = 0x01,
    MOTOR_2 = 0x02,
}MOTOR_ID;

typedef enum BOARD_STATUS_E{
    BOARD_STATUS_NONE = 0x00,
    BOARD_STATUS_ERR  = 0x01,
    BOARD_STATUS_ERR_DEVICE_NOT_DETECTED = 0x02,
    BOARD_STATUS_ERR_SOFT_VERSION = 0x03,
    BOARD_STATUS_ERR_PARAMETER = 0x04,
}BOARD_STATUS;

