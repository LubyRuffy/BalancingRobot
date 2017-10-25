import threading
import time
import math
import logging
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

        # Initialize the pigpio library
        self._pi = pigpio.pi()

        # Initialize all hardware
        self.InitMPU6050()
        self.InitMotors()
        self.InitControl()

        # Store current time
        self._currTimePrev = time.time()

        return

    def InitMPU6050(self):

        # Accelerometer Values and Device
        self._mpuIdx = ['x', 'y', 'z']
        self._a = [0.0, 0.0, 0.0] # Acceleration (g's)
        self._w = [0.0, 0.0, 0.0] # Angular rate (deg/s)
        self._wFilt = [0.0, 0.0, 0.0] # Filtered Angular rate (deg/s)
        self._theta = [0.0, 0.0, 0.0] # Angle (deg)
        self._thetaFilt = [0.0, 0.0, 0.0]  # Filtered Angle (deg)
        #self._a_off = [ 0.55,  0.08, 10.30 - 9.81]
        #self._w_off = [-6.04, -0.26,  0.25]
        self._a_off = [0.0, 0.0, 0.0]
        self._w_off = [0.0, 0.0, 0.0]
        self._thetaXFilt = ComplementaryFilter(alpha=0.02) # alpha = 0.02
        self._wXFilt = Filter(timeConst=0.3)
        #self._mpu6060 = mpu6050(0x68) # 0x68 - default I2C slave addr
        #self._mpu6060.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        #self._mpu6050.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        return

    def InitMotors(self):

        # Motor Controllers
        # Pins - PWM, FWD, REV
        self._motorLookupV = [0, 12]
        self._motorLookupPWM = [0, 100]

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
        #self._mpu6050AccelData = self._mpu6050.get_accel_data()
        #self._mpu6050GyroData = self._mpu6050.get_gyro_data()
        self._mpu6050AccelData = {'x': 0.0, 'y': 0.0, 'z': 9.81}
        self._mpu6050GyroData = {'x': 0.0, 'y': 0.0, 'z': 0.0}

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
        self._thetaFilt[0] = self._thetaXFilt.Filter(self._dT, self._w[0], self._theta[0])

        # Filter the angular velocity for controller deriviative term
        self._wFilt[0] = self._wXFilt.Filter(self._dT, self._w[0])


        print self._thetaFilt[0]

        return

    def ProcessControl(self):
        # Calculate the control error and rate
        self._ctrlErr = self._thetaFilt[0]
        self._ctrlErrRate = self._wFilt[0]

        # Run the PID controller
        self._ctrlOutput = self._pid.update(self._ctrlErr, self._ctrlErrRate)

        # Control saturation
        if self._ctrlOutput > self._ctrlOutputMax:
            self._ctrlOutput = self._ctrlOutputMax
        if self._ctrlOutput < -self._ctrlOutputMax:
            self._ctrlOutput = -self._ctrlOutputMax

        return

    def ProcessMotors(self):
        # Set directions for left and right motors
        self._ctrlLeft = self._ctrlOutput
        self._ctrlRight = self._ctrlOutput

        # Write the motor control signal
        self._motorLeft.SetVoltage(self._ctrlLeft)
        self._motorRight.SetVoltage(self._ctrlRight)

        # Process feedback from encoders
        # TODO

        return

    def run(self):
        while self._continueFlag:

            # Calculate time delta
            self._currTime = time.time()
            self._dT = self._currTimePrev - self._currTime
            self._currTimePrev = self._currTime

            # Read accelerometer and gyro data and process
            self.ProcessMPU6050()

            # Run PID Controller
            self.ProcessControl()

            # Run motor output
            self.ProcessMotors()

            logging.debug('running with %s and %s', self.args, self.kwargs)
            time.sleep(1)

        # Stop condition
        self._pi.stop()

        return

    def Stop(self):
        self._continueFlag = False

    def Reset(self):
        self._resetFlag = False


if __name__ == '__main__':
    t = RobotThread()
    t.start()
    time.sleep(5)
    t.Stop()
