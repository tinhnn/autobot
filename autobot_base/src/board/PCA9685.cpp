/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "PCA9685.h"

int pca9685_init(int busID, int address)
{
    int handle = 0;
    return handle;
}

int pca9685_deinit(int handle)
{
    int err = 0;
    return err;
}

int pca9685_set_PWM_freq(int handle, int frequency)
{
    return 0;
}

int pca9685_getspeed(int handle, int encID)
{
    return 0;
}

int pca9685_movement(int handle, uint32_t motorID, uint32_t orientation, float speed)
{
    return 0;
}
