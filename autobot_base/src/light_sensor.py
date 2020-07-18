#!/usr/bin/env python
#encoding: utf8

# Module: Light sensor

import pigpio

class Light_sensor:

    def __init__(self):
        self.board = pigpio.pi()
