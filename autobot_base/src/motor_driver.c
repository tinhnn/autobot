/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include "motor_driver.h"

#include <pigpiod_if2.h>
#include <stdio.h>
#include <stdint.h>

/* static variable */
int _i2c_hdl = 0x00;

/* prototype function */

/* API function */

/*******************************************************************************
 * @Function: motor_drv_init
 * @Brief   : board start
 * @Param   : busID: Which bus to operate
 *            address : Board controler address
 * @Return  : Board handle or error code
 *******************************************************************************
 */
int motor_drv_init(int busID, int address)
{
    int ercd = 0;
    uint8_t reg_pid = REG_PID;
    uint8_t reg_pvd = REG_PVD;
    uint8_t PID_Data;
    uint8_t VID_Data;
    uint8_t controle_mode;
    /* Init GPIO */
    if (gpioInitialise() < 0){
        /* Error log */
        return BOARD_STATUS_ERR;
    }
    /* Init I2C */
    _i2c_hdl = i2cOpen(busID, address, 0);
    if(_i2c_hdl < 0){
        /* Error log */
        return BOARD_STATUS_ERR;
    }
    
    /* Detect board */
    i2cReadI2CBlockData(_i2c_hdl, reg_pid, (char*)(&PID_Data), sizeof(PID_Data));
    i2cReadI2CBlockData(_i2c_hdl, reg_pvd, (char*)(&VID_Data), sizeof(VID_Data));
    if(PID_Data != REG_DEF_PID){
        return BOARD_STATUS_ERR_DEVICE_NOT_DETECTED;
    }
    else{
        /* Set board to DC motor */
        controle_mode = MOTOR_DC;
        i2cWriteI2CBlockData(_i2c_hdl, REG_CTRL_MODE, (char*)(&controle_mode), sizeof(controle_mode));
        /* Stop motor */
        motor_drv_movement(_i2c_hdl, MOTOR_ALL, STOP, 0);
        /* Disable encoder */
        motor_drv_set_encoder_disable(_i2c_hdl, MOTOR_ALL);
    }
    
    
    return _i2c_hdl;
}

/*******************************************************************************
 * @Function: motor_drv_deinit
 * @Brief   : board stop
 * @Param   : handle: Board handle
 * @Return  : Error Code
 *******************************************************************************
 */
int motor_drv_deinit(int handle)
{
    int ercd = 0;
    if(handle != _i2c_hdl){
        /* Not matching handle */
        return BOARD_STATUS_ERR_PARAMETER;
    }
    /* Deinit I2C */
    ercd = i2cClose(handle);
    if(ercd < 0){
        /* Error log */
        return ercd;
    }
    /* clear handle */
    _i2c_hdl = 0;
    
    /* Deinit GPIO */
    gpioTerminate();
    
    return ercd;
}

/*******************************************************************************
 * @Function: motor_drv_set_encoder_enable
 * @Brief   : Set dc motor encoder enable
 * @Param   : handle: Board handle
 *            encID : Encoder list, items in range 1 to 2, or ALL
 * @Return  : None
 *******************************************************************************
 */
void motor_drv_set_encoder_enable(int handle, int encID)
{
    char TX_Data = 1;
    uint8_t addr;
    if(encID == MOTOR_ALL){
        i2cWriteI2CBlockData(handle, REG_ENCODER1_EN, (char*)(&TX_Data), sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_ENCODER2_EN, (char*)(&TX_Data), sizeof(TX_Data));
    }
    else{
        addr = REG_ENCODER1_EN + 5*(encID-1);
        i2cWriteI2CBlockData(handle, addr, (char*)(&TX_Data), sizeof(TX_Data));
    }
}

/*******************************************************************************
 * @Function: motor_drv_set_encoder_disable
 * @Brief   : Set dc motor encoder disable
 * @Param   : handle: Board handle
 *            encID : Encoder list, items in range 1 to 2, or ALL
 * @Return  : None
 *******************************************************************************
 */
void motor_drv_set_encoder_disable(int handle, int encID)
{
    char TX_Data = 0;
    uint8_t addr;
    if(encID == MOTOR_ALL){
        i2cWriteI2CBlockData(handle, REG_ENCODER1_EN, (char*)(&TX_Data), sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_ENCODER2_EN, (char*)(&TX_Data), sizeof(TX_Data));
    }
    else{
        addr = REG_ENCODER1_EN + 5*(encID-1);
        i2cWriteI2CBlockData(handle, addr, (char*)(&TX_Data), sizeof(TX_Data));
    }
}

/*******************************************************************************
 * @Function: motor_drv_set_enc_reduct_ratio
 * @Brief   : Set dc motor encoder reduction ratio
 * @Param   : handle: board handle
 *            encID : Encoder list, items in range 1 to 2, or ALL
 *            reduction_ratio: Set dc motor encoder reduction ratio, range in 1 to 2000,
 *                             (pulse per circle) = 16 * reduction_ratio * 2
 * @Return  : Error Code
 *******************************************************************************
 */
int motor_drv_set_enc_reduct_ratio(int handle, int encID, uint16_t reduction_ratio)
{
    uint8_t TX_Data[2];
    uint8_t addr;
    if(reduction_ratio < 1 || reduction_ratio > 2000){
        /* not support reduction ratio */
        return BOARD_STATUS_ERR_PARAMETER;
    }
    
    TX_Data[0] = (reduction_ratio>>8)&0xFF;
    TX_Data[0] = reduction_ratio&0xFF;
    if(encID == MOTOR_ALL){
        i2cWriteI2CBlockData(handle, REG_ENCODER1_REDUCTION_RATIO, (char*)(&TX_Data), sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_ENCODER2_REDUCTION_RATIO, (char*)(&TX_Data), sizeof(TX_Data));
    }
    else{
        addr = REG_ENCODER1_REDUCTION_RATIO + 5*(encID-1);
        i2cWriteI2CBlockData(handle, addr, (char*)(&TX_Data), sizeof(TX_Data));
    }
    
    return BOARD_STATUS_NONE;
}

/*******************************************************************************
 * @Function: motor_drv_getspeed
 * @Brief   : Get dc motor encoder speed, unit [rpm]
 * @Param   : handle: board handle
 *            encID : Encoder list, items in range 1 to 2, or ALL
 * @Return  : encoder speed
 *******************************************************************************
 */
uint16_t motor_drv_getspeed(int handle, int encID)
{
    char RX_Data[2];
    uint8_t addr;
    uint16_t speed = 0;
    if(encID < MOTOR_1 || encID > MOTOR_2){
        // Not support get speed of all encoder
        return 0;
    }
    else{
        addr = REG_ENCODER1_SPPED + 5*(encID-1);
        i2cReadI2CBlockData(handle, addr, (char*)(&RX_Data), sizeof(RX_Data));
        speed = RX_Data[0]<<8 | RX_Data[1];
        if(speed & 0x8000){
            speed = -(0x10000 - speed);
        }
    }
    
    return speed;
}

/*******************************************************************************
 * @Function: motor_drv_set_PWM_freq
 * @Brief   : Set dc motor pwm frequency
 * @Param   : handle: board handle
 *            frequency: Frequency to set, in range 100HZ to 12750HZ, otherwise no effective
 *                       (actual frequency) = frequency - (frequency % 50)
 * @Return  : None
 *******************************************************************************
 */
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
}

/*******************************************************************************
 * @Function: motor_drv_movement
 * @Brief   : Motor movement
 * @Param   : handle: board handle
 *            motorID: Motor ID, items in range 1 to 2, or ALL
 *            orientation: Motor orientation, CW (clockwise) or CCW (counterclockwise)
 *            speed: Motor pwm duty cycle, in range 0 to 100, otherwise no effective
 * @Return  : Error code
 *******************************************************************************
 */
int motor_drv_movement(int handle, uint32_t motorID, uint32_t orientation, float speed)
{
    uint8_t TX_Data[2];
    uint8_t addr = 0;
    if(orientation < CW || orientation > STOP){
        return BOARD_STATUS_ERR_PARAMETER;
    }
    if(speed < 0 || speed > 100){
        return BOARD_STATUS_ERR_PARAMETER;
    }
    
    if(motorID == MOTOR_ALL){
        TX_Data[0] = orientation & 0xFF;
        i2cWriteI2CBlockData(handle, REG_MOTOR1_ORIENTATION, (char*)(&TX_Data), 1);
        i2cWriteI2CBlockData(handle, REG_MOTOR2_ORIENTATION, (char*)(&TX_Data), 1);
        TX_Data[0] = ((int)speed) & 0xFF;
        TX_Data[1] = ((int)(speed*10))%10;
        i2cWriteI2CBlockData(handle, REG_MOTOR1_SPEED, (char*)(&TX_Data), sizeof(TX_Data));
        i2cWriteI2CBlockData(handle, REG_MOTOR2_SPEED, (char*)(&TX_Data), sizeof(TX_Data));
    }
    else{
        addr = REG_MOTOR1_ORIENTATION + 3*(motorID-1);
        TX_Data[0] = orientation & 0xFF;
        i2cWriteI2CBlockData(handle, addr, (char*)(&TX_Data), 1);
        TX_Data[0] = ((int)speed) & 0xFF;
        TX_Data[1] = ((int)(speed*10))%10;
        i2cWriteI2CBlockData(handle, addr+1, (char*)(&TX_Data), sizeof(TX_Data));
    }
    
    return BOARD_STATUS_NONE;
}
