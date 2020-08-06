/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* Define motor control board */
#define _BOARD_PCA9685_

typedef enum MOTOR_ID_E{
    MOTOR_1 = 0x01,
    MOTOR_2 = 0x02,
    MOTOR_ALL = 0xFFFFFFFF,
}MOTOR_ID;

typedef enum MOTOR_ORIENT_E{
    CW      = 0x01,         /* orientation clockwise */
    CCW     = 0x02,         /* orientation counter clockwise */
    STOP    = 0x03,         /* motor stop */
}MOTOR_ORIENT;

typedef struct board_param_t
{
    uint32_t pwm_frq;                   /* PWM frequency */
    uint32_t enc_pulsepercycle;         /* pulse per cycle encoder */
} board_param;

typedef struct motor_param_t
{
    MOTOR_ID        motor_ID;
    MOTOR_ORIENT    motor_orient;
    double          motor_duty;
} motor_param;



int motor_drv_init(int busID, int board_addr);
int motor_drv_deinit(int handle);
int motor_drv_set_param(int handle, board_param *param);
int motor_drv_get_speed(int handle, MOTOR_ID motorID);
int motor_drv_set_speed(int handle, motor_param *motor);


#endif /* MOTOR_DRIVER_H */
