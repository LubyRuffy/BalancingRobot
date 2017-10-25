import pigpio


class MotorControl_pigpio:
    PWM_FREQ = 30.0
    PWM_DEFAULT_SPD = 0.0

    def __init__(self, pi, pinPWM, pinFWD, pinREV):
        self._pi = pi
        self._pinPWM = pinPWM
        self._pinFWD = pinFWD
        self._pinREV = pinREV

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
        if V > 0.0:
            self._pi.write(self._pinFWD, 1)
            self._pi.write(self._pinREV, 0)
        if V < 0.0:
            self._pi.write(self._pinFWD, 0)
            self._pi.write(self._pinREV, 1)
        pwm = self._lookupTable.Lookup(abs(V))
        self._pi.set_PWM_dutycycle(self._pinPWM, pwm)

    def Fwd(self):
        self._pi.write(self._pinFWD, 1)
        self._pi.write(self._pinREV, 0)

    def Rev(self):
        self._pi.write(self._pinFWD, 0)
        self._pi.write(self._pinREV, 1)

    def Brake(self):
        self._pi.write(self._pinFWD, 0)
        self._pi.write(self._pinREV, 0)

    def Float(self):
        self._pi.write(self._pinFWD, 1)
        self._pi.write(self._pinREV, 1)
