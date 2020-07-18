/* Copyright (c) 2020 Nguyen Nhan Tinh. All rights reserved. */

#include <ros/ros.h>
#include <string>

#define SOFTWARE_VERSION "0.0.1"
#define HARDWARE_VERSION "2020.08.xx"
#define FIRMWARE_VERSION_MAJOR_NUMBER 0
#define FIRMWARE_VERSION_MINOR_NUMBER 0


int main(int argc, char **argv)
{
  ros::init(argc, argv, "autobot_diagnostic");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    // publish message
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
