#! /usr/bin/python

from mpu6050 import mpu6050
import time
import math
from ComplementaryFilter import *

#ax_off = 0.0
#ay_off = 0.0
#az_off = 0.0
#wx_off = 0.0
#wy_off = 0.0
#wz_off = 0.0

ax_off = 0.66
ay_off = -0.26
az_off = 10.22 - 9.81
wx_off = -1.07
wy_off = 0.67
wz_off = -1.55

sensor = mpu6050(0x68)
sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G) # or 4,8,16G
sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG) # or 500,1000,2000DEG

samples = 1000
counter = 0
sleep = 0.01

theta_x_gyro_deg = 0.0
alpha = 0.02
# alpha = 0.02 seems to work OK
filter = ComplementaryFilter(alpha)

current_time_prev = time.time()

while counter < samples:
    counter = counter + 1

    current_time = time.time()
    dt = current_time - current_time_prev
    current_time_prev = current_time

    accelerometer_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    ax = accelerometer_data['x'] - ax_off
    ay = accelerometer_data['y'] - ay_off
    az = accelerometer_data['z'] - az_off
    wx = gyro_data['x'] - wx_off
    wy = gyro_data['y'] - wy_off
    wz = gyro_data['z'] - wz_off

    theta_x_rad = math.atan2(ay, az)
    theta_y_rad = math.atan2(-ax, az)

    theta_x_deg = math.degrees(theta_x_rad)
    theta_y_deg = math.degrees(theta_y_rad)

    theta_x_gyro_deg = theta_x_gyro_deg + wx*dt

    #print "{0:.4f},{1:.4f},{2:.4f},{3:.4f},{4:.4f},{5:.4f}".format(\
    #    ax, ay, az, wx, wy, wz)

    print "{0:.4f},{1:.4f}".format(theta_x_deg, wx)

    #filt_val = filter.Filter(dt, wx, theta_x_deg)
    #print "{0:.4f},{1:.4f},{2:.4f},{3:.4f},{4:.4f}".format(dt, filt_val, theta_x_deg, wx, theta_x_gyro_deg)

    time.sleep(sleep)
