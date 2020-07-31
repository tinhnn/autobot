/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */


#ifndef PCA9685_H
#define PCA9685_H

int pca9685_init(int busID, int address);
int pca9685_deinit(int handle);


#endif /* PCA9685_H */
