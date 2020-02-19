#!/usr/bin/env python
#encoding: utf8

# Module: ultra sound
#

import pigpio
import time

class Ultra_sound:
    _pingPin = 23    # Trigger Pin of Ultrasonic Sensor
    _echoPin = 24    # Echo Pin of Ultrasonic Sensor
    
    def __init__(self):
        self.board = pigpio.pi()
        self.board.set_mode(_pingPin,pigpio.OUTPUT)
        self.board.set_mode(_echoPin,pigpio.INPUT)

    def get_distance:
        # Trigger pin is set low, and give the sensor a second to settle
        self.board.write(_pingPin, 0)
        time.sleep(2)
        # create a trigger pulse
        self.board.write(_pingPin, 1)
        time.sleep(0.00001)
        self.board.write(_pingPin, 0)
        
        # Read duration time
        while self.board.read(_echoPin)==0:
            pulse_start = time.time()
        while self.board.read(_echoPin)==1:
            pulse_end = time.time()
            
        pulse_duration = pulse_end - pulse_start
        
        # convert to distance (cm)
        distance = pulse_duration*17150
        distance = round(distance, 2)       # unit: cm
        
        return distance
