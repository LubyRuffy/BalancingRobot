#! /usr/bin/python

import threading
import time
import math
import logging
import socket
import pigpio

from LookupTable import LookupTable
from mpu6050 import mpu6050
from Filter import Filter
from ComplementaryFilter import ComplementaryFilter
from PID import PID
from MotorControl_pigpio import MotorControl_pigpio

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)

class RobotThread(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(RobotThread,self).__init__(group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs

        self._continueFlag = True # flag to continue running the thread
        self._resetFlag = False # flag to reset everything
        self._runControl = False # process control

        # Initialize the pigpio library
        self._pi = pigpio.pi()

        # Initialize all hardware
        self.InitMPU6050()
        self.InitMotors()
        self.InitControl()

        # Store current time
        self._currTimePrev = time.time()

        # Sleep time for control loop
        self._sleepTime = 0.1

        return

    @property
    def CtrlErr(self):
        return self._ctrlErr

    @property
    def CtrlErrRate(self):
        return self._ctrlErrRate

    @property
    def CtrlOutput(self):
        return self._ctrlOutput

    @property
    def Kp(self):
        return self._P

    @Kp.setter
    def Kp(self, val):
        self._P = val
        self._pid.setKp(val)

    @property
    def Ki(self):
        return self._I

    @Ki.setter
    def Ki(self, val):
        self._I = val
        self._pid.setKi(val)

    @property
    def Kd(self):
        return self._D

    @Kd.setter
    def Kd(self, val):
        self._D = val
        self._pid.setKd(val)

    def InitMPU6050(self):

        # Accelerometer Values and Device
        self._mpuIdx = ['x', 'y', 'z']
        self._a = [0.0, 0.0, 0.0] # Acceleration (g's)
        self._w = [0.0, 0.0, 0.0] # Angular rate (deg/s)
        self._wFilt = [0.0, 0.0, 0.0] # Filtered Angular rate (deg/s)
        self._theta = [0.0, 0.0, 0.0] # Angle (deg)
        self._thetaFilt = [0.0, 0.0, 0.0]  # Filtered Angle (deg)
        self._a_off = [ 0.66, -0.26, 10.22 - 9.81]
        self._w_off = [-1.07, 0.67, -1.55]
        #self._a_off = [0.0, 0.0, 0.0]
        #self._w_off = [0.0, 0.0, 0.0]
        self._thetaXFilt = ComplementaryFilter(alpha=0.1) # alpha = 0.02
        self._thetaXFilt2 = Filter(timeConst=0.3)
        self._wXFilt = Filter(timeConst=0.3)
        self._mpu6050 = mpu6050(0x68) # 0x68 - default I2C slave addr
        self._mpu6050.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self._mpu6050.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        return

    def InitMotors(self):

        # Motor Controllers
        # Pins - PWM, FWD, REV
        #self._motorLookupV = [0, 12]
        #self._motorLookupPWM = [0, 100]
        self._motorLookupV = [0.0, 1.0, 1.1, 10.4, 11.0, 11.5, 11.8, 12.0]
        self._motorLookupPWM = [0.0, 1.0, 2.0, 80.0, 85.0, 90.0, 95.0, 100.0]

        self._motorLookup = LookupTable(\
            self._motorLookupV, self._motorLookupPWM)

        self._motorLeft = MotorControl_pigpio(\
            self._pi, 13, 19, 26)
        self._motorRight = MotorControl_pigpio(\
            self._pi, 16, 20, 21)

        self._motorLeft.SetVoltageLookupTable(\
            self._motorLookup)
        self._motorRight.SetVoltageLookupTable(\
            self._motorLookup)

        # Motor Encoders
        # TODO

        return

    def InitControl(self):

        self._P = 0.2
        self._I = 0.0
        self._D = 0.0
        self._pid = PID(self._P, self._I, self._D)
        self._ctrlOutputMin = 1.0  # Volts, min motor output
        self._ctrlOutputMax = 12.0 # Volts, max motor output

        return

    def ProcessMPU6050(self):

        # Get the raw data
        self._mpu6050AccelData = self._mpu6050.get_accel_data()
        self._mpu6050GyroData = self._mpu6050.get_gyro_data()
        #self._mpu6050AccelData = {'x': 0.0, 'y': 0.0, 'z': 9.81}
        #self._mpu6050GyroData = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Subtract out calibration offsets
        for i in range(0, 3): # (0, 1, 2)
            self._a[i] = self._mpu6050AccelData[self._mpuIdx[i]] - \
                self._a_off[i]
            self._w[i] = self._mpu6050GyroData[self._mpuIdx[i]] - \
                self._w_off[i]

        # Calculate angle from accelerometer data
        self._theta[0] = math.degrees( \
            math.atan2(self._a[1], self._a[2])) # atan2(ay, az)

        # Complementary filter on accel and gyro data
        thetaFilt = self._thetaXFilt.Filter(self._dT, self._w[0], self._theta[0])

        # Filter the resulting angle
        self._thetaFilt[0] = self._thetaXFilt2.Filter(self._dT, thetaFilt)

        # Filter the angular velocity for controller deriviative term
        self._wFilt[0] = self._wXFilt.Filter(self._dT, self._w[0])

        return

    def ProcessControl(self):
        # Calculate the control error and rate
        self._ctrlErr = self._thetaFilt[0]
        self._ctrlErrRate = self._wFilt[0]
        #self._ctrlErrRate = self._w[0]

        # Run the PID controller
        self._ctrlOutput = self._pid.update(self._ctrlErr, self._ctrlErrRate)

        # Control saturation
        if self._ctrlOutput > self._ctrlOutputMax:
            self._ctrlOutput = self._ctrlOutputMax
        if self._ctrlOutput < -self._ctrlOutputMax:
            self._ctrlOutput = -self._ctrlOutputMax

        # Clear integrator if not running
        if not self._runControl:
            self._pid.int_error = 0.0

        return

    def ProcessMotors(self):
        # If not running control, set both motors to brake
        if not self._runControl:
            self._motorLeft.Brake()
            self._motorRight.Brake()
            return

        # Set directions for left and right motors
        self._ctrlLeft = self._ctrlOutput
        self._ctrlRight = -self._ctrlOutput

        # Write the motor control signal
        self._motorLeft.SetVoltage(self._ctrlLeft)
        self._motorRight.SetVoltage(self._ctrlRight)

        # Process feedback from encoders
        # TODO

        return

    def run(self):
        while self._continueFlag:

            #print self._runControl

            # Calculate time delta
            self._currTime = time.time()
            self._dT = self._currTime - self._currTimePrev
            self._currTimePrev = self._currTime

            # Read accelerometer and gyro data and process
            self.ProcessMPU6050()

            # Run PID Controller
            self.ProcessControl()

            # Run motor output
            self.ProcessMotors()

            #logging.debug('running with %s and %s', self.args, self.kwargs)
            time.sleep(self._sleepTime)

        # Stop condition
        self._pi.stop()

        return

    def StartControl(self):
        self._runControl = True

    def StopControl(self):
        self._runControl = False

    def Stop(self):
        self._continueFlag = False

    def Reset(self):
        self._resetFlag = False

TCP_IP = "0.0.0.0"
TCP_PORT = 9999
BUFFER_SIZE = 128

if __name__ == '__main__':
    t = RobotThread()
    t.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((TCP_IP, TCP_PORT))
    sock.listen(1)
    conn, addr = sock.accept()
    print "Connection Address:", addr

    while 1:
        data = conn.recv(BUFFER_SIZE)

        if not data:
            break

        cmd = data.split()
        print len(cmd)

        if data == "CMD START":
            t.StartControl()
        if data == "CMD STOP":
            t.StopControl()
        if data == "GET GAINS":
            resp = "RESP GAINS {0} {1} {2}".format(t.Kp, t.Ki, t.Kd)
            conn.send(resp)
        if data == "GET PARAMS":
            resp = "RESP PARAMS {0} {1} {2}".format(t.CtrlErr, t.CtrlErrRate, t.CtrlOutput)
            conn.send(resp)

        if cmd[0] == "SET":
            if cmd[1] == "GAINS":
                t.Kp = float(cmd[2])
                t.Ki = float(cmd[3])
                t.Kd = float(cmd[4])
                resp = "RESP GAINS {0} {1} {2}".format(t.Kp, t.Ki, t.Kd)
                conn.send(resp)

    t.Stop()
