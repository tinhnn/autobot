/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef DRF0592_H
#define DRF0592_H

#include <stdio.h>
#include <stdint.h>

/* Board address: ex: 0x10 */
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

/* Board controle mode */
typedef enum CONTROL_MODE_E{
    MOTOR_DC        = 0x00,
    MOTOR_STEPPER   = 0x01,
}CONTROL_MODE;

/* Orientation  */
typedef enum MOTOR_ORIENT_E{
    CW      = 0x01,         /* orientation clockwise */
    CCW     = 0x02,         /* orientation counter clockwise */
    STOP    = 0x03,         /* motor stop */
}MOTOR_ORIENT;

typedef enum MOTOR_ID_E{
    MOTOR_1 = 0x01,
    MOTOR_2 = 0x02,
    MOTOR_ALL = 0xFFFFFFFF,
}MOTOR_ID;

typedef enum BOARD_STATUS_E{
    BOARD_STATUS_NONE = 0x00,
    BOARD_STATUS_ERR  = -1,
    BOARD_STATUS_ERR_DEVICE_NOT_DETECTED = -2,
    BOARD_STATUS_ERR_SOFT_VERSION = -3,
    BOARD_STATUS_ERR_PARAMETER = -4,
}BOARD_STATUS;

/* Motor driver API function */
int drf0592_init(int busID, int address);
int drf0592_deinit(int handle);
void drf0592_set_encoder_enable(int handle, int encID);
void drf0592_set_encoder_disable(int handle, int encID);
int drf0592_set_enc_reduct_ratio(int handle, int encID, uint16_t reduction_ratio);
uint16_t drf0592_getspeed(int handle, int encID);
int drf0592_set_PWM_freq(int handle, int frequency);
int drf0592_movement(int handle, uint32_t motorID, uint32_t orientation, float speed);


#endif /* DRF0592_H */
