/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include <cstdio>

#include "motor_driver.h"

#ifdef _BOARD_DRF0592_
#include "board/DRF0592.h"

#define boar_init(busID, address)   drf0592_init(busID, address)
#define boar_deinit(handle)         drf0592_deinit(handle)
#define set_speed(handle, motorID, orientation, speed) drf0592_movement(handle, motorID, orientation, speed)
#endif

#ifdef _BOARD_PCA9685_
#include "board/PCA9685.h"

#define boar_init(busID, address)               pca9685_init(busID, address)
#define boar_deinit(handle)                     pca9685_deinit(handle)
#define set_pwm_frq(handle, frequency)          pca9685_set_PWM_freq(handle, frequency);
#define set_enc_ratio(handle, ratio)            0
#define get_speed(handle, motorID)              pca9685_getspeed(handle, motorID)
#define set_speed(handle, motorID, orient, spd) pca9685_movement(handle, motorID, orient, spd)
#endif

int motor_drv_init(int busID, int board_addr)
{
    int handle = 0;
    int err = 0;
    err = boar_init(busID, board_addr);
    if(err <= 0){
        printf("Init Board fail [busID = %d] [address = 0x%f]",busID,board_addr);
        return err;
    }
    else{
        handle = err;
    }

    #ifdef _BOARD_DRF0592_
    drf0592_set_encoder_enable(handle, 1);
    drf0592_set_encoder_enable(handle, 2);
    #endif

    return handle;
}

int motor_drv_deinit(int handle)
{
    int err = 0;
    err = boar_deinit(handle);
    return err;
}

int motor_drv_set_param(int handle, board_param *param)
{
    int err = 0;
    /* Set pwm frequency */
    err |= set_pwm_frq(handle, param->pwm_frq);
    /* Set encoder reduction ratio */
    err |= set_enc_ratio(handle, param->enc_pulsepercycle);
    return err;
}

int motor_drv_get_speed(int handle, MOTOR_ID motorID)
{
    int err = 0;    
    err = get_speed(handle, motorID);    
    return err;
}

int motor_drv_set_speed(int handle, motor_param *motor)
{
    int err = 0;
    err = set_speed(handle, motor->motor_ID, motor->motor_orient, motor->motor_duty);
    return err;
}

