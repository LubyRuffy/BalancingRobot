#! /usr/bin/python

import pigpio
import time
import math
from MotorControl_pigpio import *
from LookupTable import *

ENA_1_PIN_BCM = 13
IN1_PIN_BCM = 19
IN2_PIN_BCM = 26
ENA_2_PIN_BCM = 16
IN3_PIN_BCM = 20
IN4_PIN_BCM = 21
ENA_FREQ = 30

SINE_AMP = 10.0
SINE_PERIOD = 0.5
SLEEP_TIME = 0.01

PWM = [0.0, 100.0]
V = [0.0, 12.0]
table = LookupTable(V, PWM)


pi = pigpio.pi()

mr = MotorControl_pigpio(pi, ENA_1_PIN_BCM, IN1_PIN_BCM, IN2_PIN_BCM)
ml = MotorControl_pigpio(pi, ENA_2_PIN_BCM, IN3_PIN_BCM, IN4_PIN_BCM)

mr.SetVoltageLookupTable(table)
ml.SetVoltageLookupTable(table)

mr.Fwd()
ml.Fwd()

lastTime = time.time()
currTime = lastTime
initTime = lastTime

while 1:
    currTime = time.time()
    T = currTime - initTime
    #print T

    val = SINE_AMP * math.sin(2*math.pi * T/SINE_PERIOD)
    print val

    ml.SetVoltage(val)
    mr.SetVoltage(val)

    time.sleep(SLEEP_TIME)
