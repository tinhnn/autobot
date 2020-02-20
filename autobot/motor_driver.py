#!/usr/bin/env python
#encoding: utf8

# Module: motor_driver
#       -------------------
#      *|PWMA      motorA+|*
#      *|AI1       motorA-|*
#      *|AI2              |
#      *|Enable           |
#      *|BI2              |
#      *|BI1       motorB+|*
#      *|PWMB      motorB-|*
#       -------------------

import pigpio

class Motor_drv:
    default_freq = 1000     # PWM frequency (default = 1000Hz)
    default_range = 100     # duty range (default = 100%)

    def __init__(self,pPWM,pAI1,pAI2,pEn):
        self.board = pigpio.pi()

        self._pPWM = pPWM
        self._pAI1 = pAI1
        self._pAI2 = pAI2
        self._pEn  = pEn
        self._freq = self.default_freq
        self._drange = self.default_range

        self.board.set_mode(self._pPWM,pigpio.OUTPUT)
        self.board.set_mode(self._pAI1,pigpio.OUTPUT)
        self.board.set_mode(self._pAI2,pigpio.OUTPUT)
        self.board.set_PWM_frequency(self._pPWM,self._freq)
        self.board.set_PWM_range(self._pPWM,self._drange)
        self.board.set_PWM_dutycycle(self._pPWM,0)
        if pEn > 0:     # if used pin enable motor driver
            self.board.set_mode(self._pEn,pigpio.OUTPUT)
            self.board.write(self._pEn,1)

    def set_enable(self,onoff)
        self.board.write(self._pEn,onoff)

    def set_velocity(self,_dir,duty):
        # set direction
        if _dir == 'F':     # forward
            self.board.write(self._pAI1,1)
            self.board.write(self._pAI2,0)
        elif _dir == 'R':   # reverse
            self.board.write(self._pAI1,0)
            self.board.write(self._pAI2,1)
        elif _dir == 'B':   # brake
            self.board.write(self._pAI1,1)
            self.board.write(self._pAI2,1)
        elif _dir == 'S':   # stop
            self.board.write(self._pAI1,0)
            self.board.write(self._pAI2,0)
        # set duty
        if duty <= 1:
            self.board.set_PWM_dutycycle(self._pPWM,(int)(self._drange*duty))

    def get_frequency(self,freq):
        return self._freq

    def set_frequency(self,freq):
        self._freq = freq
        self.board.set_PWM_frequency(self._pPWM,freq)

    def get_range(self,drange):
        return self._drange

    def set_range(self,drange):
        self._drange = drange
        self.pi_set_PWM_range(self._pPWM,drange)

    def set_duty(self,duty):
        self.board.set_PWM_dutycycle(self._pPWM,(int)(self._drange*duty))
