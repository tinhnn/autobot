/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

/* Define motor control board */
#define _BOARD_PCA9685_

typedef struct motor_drv_param_t
{
    uint32_t pwm_frq;                   /* PWM frequency */
    uint32_t enc_pulsepercycle;         /* pulse per cycle encoder */
} motor_drv_param;


int motor_drv_init(int busID, int board_addr);
int motor_drv_deinit(int handle);
int motor_drv_set_param(int handle, motor_drv_param *param);
int motor_drv_set_speed(int handle, uint32_t encID);
int motor_drv_get_speed(int handle, uint32_t motorID, uint32_t orientation, float speed);


#endif /* MOTOR_DRIVER_H */
