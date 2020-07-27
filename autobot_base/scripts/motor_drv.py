#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import sin, cos, tan, pi
from utils.Adafruit_MotorHAT_Motors import Adafruit_MotorHAT

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
    max_pwm = 115.0
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == 1:
        motor = motor_left
    elif motor_ID == 2:
        motor = motor_right
    else:
        rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
        return
    
    motor.setSpeed(speed)

    if value > 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)

# Convert Twist[linear, angl] to duty of PWM
def twist2duty(twist):
    max_rpm = 127.39        # (max rotation speed/Gear ratio = 6000/47.1)
    axle_length = 0.12
    R_Wheel = 0.03

    vcmd = twist.linear.x
    omega = twist.angular.z

    v_l = (2*vcmd - omega*axle_length)/2
    v_r = (2*vcmd + omega*axle_length)/2    

    v_l = v_l/(R_Wheel * 2 * pi)     # wheel speed (rps)
    v_r = v_r/(R_Wheel * 2 * pi)     # wheel speed (rps)
    # duty [0.0 ~ 1.0]
    l_duty = 60*v_l/max_rpm
    r_duty = 60*v_r/max_rpm

    return l_duty, r_duty
# Processing command velocity
def on_cmd_vel(msg):
    twist = msg
    L_duty, R_duty = twist2duty(twist)
    set_speed(motor_left_ID,  L_duty)
    set_speed(motor_right_ID,  R_duty) 


# initialization
if __name__ == '__main__':

    # setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2

    motor_left = motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)

    # stop the motors as precaution
    all_stop()

    # setup ros node
    rospy.init_node('autobot_motors')
    
    rospy.Subscriber('~cmd_vel', Twist,   on_cmd_vel, queue_size=1)                 # Subscribe velocity from Matlab

    # start running
    rospy.spin()

    # stop motors before exiting
    all_stop()