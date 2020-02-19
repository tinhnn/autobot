#!/usr/bin/env python
#encoding: utf8

# Module: Encoder
#

import pigpio, struct

class Encoder:
    def __init__(self):
        self.board = pigpio.pi()
