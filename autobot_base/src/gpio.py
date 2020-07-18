#!/usr/bin/env python
#encoding: utf8

# Module: gpio
#

import pigpio

class GPIO:
    def __init__(self):
        self.board = pigpio.pi()

def set_mode(self,pin, mode)
        self.board.set_mode(pin,mode)

def get_mode(self,pin)
        return self.board.get_mode(pin)

def read(self,pin)
        return self.board.read(pin)

def write(self,pin,val)
        self.board.write(pin, val)

def stop(self)
        self.board.stop()