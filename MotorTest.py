#! /usr/bin/python

import RPi.GPIO as GPIO
import time
import math

IN1_PIN_BCM = 6
IN2_PIN_BCM = 13
ENA_1_PIN_BCM = 22
IN3_PIN_BCM = 16
IN4_PIN_BCM = 20
ENA_2_PIN_BCM = 21
ENA_FREQ = 30

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1_PIN_BCM, GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
GPIO.setup(IN2_PIN_BCM, GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
GPIO.setup(IN3_PIN_BCM, GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
GPIO.setup(IN4_PIN_BCM, GPIO.OUT)
GPIO.setup(ENA_1_PIN_BCM, GPIO.OUT)
GPIO.setup(ENA_2_PIN_BCM, GPIO.OUT)
ENA_1 = GPIO.PWM(ENA_1_PIN_BCM, ENA_FREQ)
ENA_2 = GPIO.PWM(ENA_2_PIN_BCM, ENA_FREQ)

current_time_prev = time.time()

default_spd = 0.0

ENA_1.start(default_spd)
ENA_2.start(default_spd)

while 1:
    current_time = time.time()
    dt = current_time - current_time_prev
    current_time_prev = current_time

    GPIO.output(IN1_PIN_BCM, GPIO.HIGH)
    GPIO.output(IN2_PIN_BCM, GPIO.LOW)
    #GPIO.output(ENA_1_PIN_BCM, GPIO.HIGH)

    spd = int(input("Spd (0-100%): "))
    ENA_1.ChangeDutyCycle(spd)
    ENA_2.ChangeDutyCycle(spd)
    time.sleep(1)
