#!/usr/bin/env python
#encoding: utf8

# Module: Infrared

import pigpio

class IR_Hasher:
    def __init__(self, gpio, callback, timeout=5):
        self.pi = pigpio.pi()
        self.gpio = gpio
        self.code_timeout = timeout
        self.callback = callback
        self.in_code = False

        self.pi.set_mode(gpio, pigpio.INPUT)
        self.cb = self.pi.callback(gpio, pigpio.EITHER_EDGE, self._cb)

    def _hash(self, old_val, new_val):

        if   new_val < (old_val * 0.60):
            val = 13
        elif old_val < (new_val * 0.60):
            val = 23
        else:
            val = 2

      self.hash_val = self.hash_val ^ val
      self.hash_val *= 16777619 # FNV_PRIME_32
      self.hash_val = self.hash_val & ((1<<32)-1)

    def _cb(self, gpio, level, tick):
        if level != pigpio.TIMEOUT:
            if self.in_code == False:
                self.in_code = True
                self.pi.set_watchdog(self.gpio, self.code_timeout)
                self.hash_val = 2166136261 # FNV_BASIS_32
                self.edges = 1
                self.t1 = None
                self.t2 = None
                self.t3 = None
                self.t4 = tick
            else:
                self.edges += 1
                self.t1 = self.t2
                self.t2 = self.t3
                self.t3 = self.t4
                self.t4 = tick
                if self.t1 is not None:
                    d1 = pigpio.tickDiff(self.t1,self.t2)
                    d2 = pigpio.tickDiff(self.t3,self.t4)
                    self._hash(d1, d2)
        else:
            if self.in_code:
                self.in_code = False
                self.pi.set_watchdog(self.gpio, 0)
                if self.edges > 12:
                    self.callback(self.hash_val)
