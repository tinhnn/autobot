/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdio>

#include "motor_driver.h"

#define PCA9685_ADDR 0x60
#define DFR0592_ADDR 0x10
#define axle_length  0.12
#define R_Wheel      0.03

int _motor_handle;



void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    double linear_x;
    double angle_z;
    motor_param motor_l;
    motor_param motor_r;
    double max_rpm = 127.39;
    double pi = 3.1415926;

    motor_l.motor_ID = MOTOR_1;
    motor_r.motor_ID = MOTOR_2;
    linear_x = msg.linear.x;
    angle_z = msg.angular.z;

    /* Convert twist to duty */
    double v_l = (2*linear_x - angle_z*axle_length)/2;
    double v_r = (2*linear_x + angle_z*axle_length)/2;
    v_l = v_l/(R_Wheel * 2 * pi);     // wheel speed (rps)
    v_r = v_r/(R_Wheel * 2 * pi);     // wheel speed (rps)

    motor_l.motor_duty = 60*v_l/max_rpm;
    motor_r.motor_duty = 60*v_r/max_rpm;

    motor_l.motor_orient = motor_l.motor_duty>=0?CW:CCW;
    motor_r.motor_orient = motor_r.motor_duty>=0?CW:CCW;

    /* Set velocity to motor */
    motor_drv_set_speed(_motor_handle, &motor_l);
    motor_drv_set_speed(_motor_handle, &motor_r);
}


// node main loop
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "autobot_base");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    /* Init motor driver */
    // PCA9685 address = 0x60
    // DFR0592 address = 0x10
    _motor_handle = motor_drv_init(1, PCA9685_ADDR);

    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &cmd_vel_callback);
    
    while( ros::ok() )
    {
        ros::spinOnce();
    }

    motor_drv_deinit(_motor_handle);

    return 0;
}
