import pigpio
import time

class MotorControl_pigpio:
    PWM_FREQ = 30.0
    PWM_DEFAULT_SPD = 0.0
    ZERO_STATE_HOLD = 0.1

    STATE_REV = -1
    STATE_ZERO = 0
    STATE_FWD = 1

    def __init__(self, pi, pinPWM, pinFWD, pinREV):
        self._pi = pi
        self._pinPWM = pinPWM
        self._pinFWD = pinFWD
        self._pinREV = pinREV
        self._state = MotorControl_pigpio.STATE_ZERO # current direction
        self._lastState = MotorControl_pigpio.STATE_ZERO # last direction
        self._lastDir = 0.0
        self._lastZeroTime = time.time()

        pi.set_mode(self._pinPWM, pigpio.OUTPUT)
        pi.set_mode(self._pinFWD, pigpio.OUTPUT)
        pi.set_mode(self._pinREV, pigpio.OUTPUT)
        pi.write(self._pinFWD, 0)
        pi.write(self._pinREV, 0)

        pi.set_PWM_frequency(self._pinPWM, MotorControl_pigpio.PWM_FREQ)
        pi.set_PWM_range(self._pinPWM, 100)
        pi.set_PWM_dutycycle(self._pinPWM, MotorControl_pigpio.PWM_DEFAULT_SPD)

    def SetVoltageLookupTable(self, table):
        self._lookupTable = table

    def SetPwmDutyCycle(self, pwm):
        if pwm < 0.0:
            pwm = 0.0
        if pwm > 100.0:
            pwm = 100.0

        self._pi.set_PWM_dutycycle(self._pinPWM, pwm)


    def SetVoltage(self, V):
        self.SetDirection(V)
        pwm = self._lookupTable.Lookup(abs(V))
        self.SetPwmDutyCycle(pwm)

    def SetDirection(self, dir):
        currDir = self.Sign(dir)
        currTime = time.time()
        dZeroTime = currTime - self._lastZeroTime

        if (currDir == self._lastDir) and \
            (dZeroTime > MotorControl_pigpio.ZERO_STATE_HOLD):
            if dir > 0.0:
                self.Fwd()
            elif dir < 0.0:
                self.Rev()
            else:
                self.Brake()
                self._lastZeroTime = currTime
        else:
            self.Brake()
            if self._lastState != MotorControl_pigpio.STATE_ZERO:
                self._lastZeroTime = currTime

        self._lastDir = currDir
        self._lastState = self._state

    def Sign(self, val):
        if val > 0.0:
            return 1.0
        elif val < 0.0:
            return -1.0
        elif val == 0.0:
            return 0.0
        else:
            return val

    def Fwd(self):
        self._pi.write(self._pinFWD, 1)
        self._pi.write(self._pinREV, 0)
        self._state = MotorControl_pigpio.STATE_FWD

    def Rev(self):
        self._pi.write(self._pinFWD, 0)
        self._pi.write(self._pinREV, 1)
        self._state = MotorControl_pigpio.STATE_REV 

    def Brake(self):
        self._pi.write(self._pinFWD, 0)
        self._pi.write(self._pinREV, 0)
        self._state = MotorControl_pigpio.STATE_ZERO

    def Float(self):
        self._pi.write(self._pinFWD, 1)
        self._pi.write(self._pinREV, 1)
        self._state = MotorControl_pigpio.STATE_ZERO
