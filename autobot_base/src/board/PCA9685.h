/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */


#ifndef PCA9685_H
#define PCA9685_H

#include <stdio.h>
#include <stdint.h>

int pca9685_init(int busID, int address);
int pca9685_deinit(int handle);
int pca9685_set_PWM_freq(int handle, int frequency);
int pca9685_getspeed(int handle, int encID);
int pca9685_movement(int handle, uint32_t motorID, uint32_t orientation, float speed);

#endif /* PCA9685_H */
