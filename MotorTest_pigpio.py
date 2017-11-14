#! /usr/bin/python

import pigpio
import time
import math
from MotorControl_pigpio import *

ENA_1_PIN_BCM = 13
IN1_PIN_BCM = 19
IN2_PIN_BCM = 26
ENA_2_PIN_BCM = 16
IN3_PIN_BCM = 20
IN4_PIN_BCM = 21
ENA_FREQ = 30

pi = pigpio.pi()

mr = MotorControl_pigpio(pi, ENA_1_PIN_BCM, IN1_PIN_BCM, IN2_PIN_BCM)
ml = MotorControl_pigpio(pi, ENA_2_PIN_BCM, IN3_PIN_BCM, IN4_PIN_BCM)

mr.Fwd()
ml.Fwd()

while 1:
    spd = int(input("Spd (0-100%): "))

    if spd > 0:
        mr.Fwd()
        ml.Fwd()
    else:
        mr.Rev()
        ml.Rev()

    ml.SetPwmDutyCycle(math.fabs(spd))
    mr.SetPwmDutyCycle(math.fabs(spd))

    time.sleep(1)
