#!/usr/bin/env python
#encoding: utf8

# Module: stepmotor_driver

import pigpio

class Stepmotor_drv:

    def __init__(self):
        self.board = pigpio.pi()
