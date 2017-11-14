#! /usr/bin/python

import pigpio
import time
import math

ENA_1_PIN_BCM = 13
IN1_PIN_BCM = 19
IN2_PIN_BCM = 26
ENA_2_PIN_BCM = 16
IN3_PIN_BCM = 20
IN4_PIN_BCM = 21
ENA_FREQ = 30

pi = pigpio.pi()
pi.set_mode(IN1_PIN_BCM, pigpio.OUTPUT)
pi.set_mode(IN2_PIN_BCM, pigpio.OUTPUT)
pi.set_mode(IN3_PIN_BCM, pigpio.OUTPUT)
pi.set_mode(IN4_PIN_BCM, pigpio.OUTPUT)
pi.set_mode(ENA_1_PIN_BCM, pigpio.OUTPUT)
pi.set_mode(ENA_2_PIN_BCM, pigpio.OUTPUT)

current_time_prev = time.time()

default_spd = 0.0


pi.set_PWM_frequency(ENA_1_PIN_BCM, ENA_FREQ)
pi.set_PWM_frequency(ENA_2_PIN_BCM, ENA_FREQ)

pi.set_PWM_range(ENA_1_PIN_BCM, 100)
pi.set_PWM_range(ENA_2_PIN_BCM, 100)

while 1:
    current_time = time.time()
    dt = current_time - current_time_prev
    current_time_prev = current_time

    pi.write(IN1_PIN_BCM, 1)
    pi.write(IN2_PIN_BCM, 0)

    pi.write(IN3_PIN_BCM, 1)
    pi.write(IN4_PIN_BCM, 0)

    spd = int(input("Spd (0-100%): "))

    pi.set_PWM_dutycycle(ENA_1_PIN_BCM, spd)
    pi.set_PWM_dutycycle(ENA_2_PIN_BCM, spd)

    time.sleep(1)

