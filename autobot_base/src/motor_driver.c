/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "motor_driver.h"

#include "pigpio.h"
#include <stdio.h>
#include <stdint.h>

/* static variable */
int _i2c_hdl = 0x00;

/* static function */

int motor_drv_init(int busID, int address)
{
    ercd = 0;
    /* Init GPIO */
    ercd = gpioInitialise();
    if (ercd < 0){
        /* Error log */
        return ercd;
    }
    /* Init I2C */
    _i2c_hdl = i2cOpen(busID, address, 0);
    if(_i2c_hdl < 0){
        /* Error log */
        return _i2c_hdl;
    }
    
    return 0;
}

int motor_drv_deinit(int handle)
{
    int ercd;
    if(handle != _i2c_hdl){
        /* Not matching handle */
        return -1;
    }
    /* Deinit I2C */
    ercd = i2cClose(handle);
    if(ercd < 0){
        /* Error log */
        return ercd;
    }
    
    /* Deinit GPIO */
    ercd = gpioTerminate();
    return ercd;
}

int motor_drv_set_encoder_enable(int handle, int encID)
{
    char TX_Data = 1;
    uint8_t addr;
    if(encID == MOTOR_ALL){
        i2cWriteI2CBlockData(handle, REG_ENCODER1_EN, &TX_Data, sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_ENCODER2_EN, &TX_Data, sizeof(TX_Data));
    }
    else{
        addr = REG_ENCODER1_EN + 5*(encID-1);
        i2cWriteI2CBlockData(handle, addr, &TX_Data, sizeof(TX_Data));
    }
}

int motor_drv_set_encoder_disable(int handle, int encID)
{
    char TX_Data = 0;
    uint8_t addr;
    if(encID == MOTOR_ALL){
        i2cWriteI2CBlockData(handle, REG_ENCODER1_EN, &TX_Data, sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_ENCODER2_EN, &TX_Data, sizeof(TX_Data));
    }
    else{
        addr = REG_ENCODER1_EN + 5*(encID-1);
        i2cWriteI2CBlockData(handle, addr, &TX_Data, sizeof(TX_Data));
    }
}

uint16_t motor_drv_getspeed(int handle, int encID)
{
    char RX_Data[2];
    uint8_t addr;
    uint16_t speed = 0;
    if(encID == MOTOR_ALL){
        return 0;
    }
    else{
        addr = REG_ENCODER1_SPPED + 5*(encID-1);
        i2cReadI2CBlockData(handle, addr, &RX_Data, sizeof(TX_Data));
        speed = RX_Data[0]<<8 | RX_Data[1];
        if(speed&0x8000){
            speed = -(0x10000 - speed);
        }
    }
    
    return speed;
}

int motor_drv_set_PWM_freq(int handle, int frequency)
{
    int ercd;
    int frq;
    if (frequency < 100 || frequency > 12750){
        /* not support frequency */
        return BOARD_STATUS_ERR_PARAMETER;
    }
    
    frq = frequency/50;
    i2cWriteI2CBlockData(handle, REG_MOTOR_PWM, (char*)(&frq), sizeof(frq));
    return 0;
}

int motor_drv_movement(int handle, int motorID, double orientation, double speed)
{
    return 0;
}
