/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "motor_driver.h"

#ifdef _BOARD_DRF0592_
#include "board/DRF0592.h"
#endif

#ifdef _BOARD_PCA9685_
#include "board/PCA9685.h"
#endif

int motor_drv_init(int busID, int board_addr)
{
    int handle = 0;

    return handle;
}

int motor_drv_deinit(int handle)
{
    int err = 0;
    return err;
}

int motor_drv_set_param(int handle, motor_drv_param *param)
{
    int err = 0;
    return err;
}

int motor_drv_set_speed(int handle, uint32_t encID)
{
    int err = 0;
    return err;
}

int motor_drv_get_speed(int handle, uint32_t motorID, uint32_t orientation, float speed)
{
    int err = 0;
    return err;
}

