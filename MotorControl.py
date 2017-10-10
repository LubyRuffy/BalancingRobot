import RPi.GPIO as GPIO

class MotorControl:
    PWM_FREQ = 30.0
    PWM_DEFAULT_SPD = 0.0

    def __init__(self, pinPWM, pinFWD, pinREV):
        self._pinPWM = pinPWM
        self._pinFWD = pinFWD
        self._pinREV = pinREV
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pinPWM, GPIO.OUT)
        GPIO.setup(self._pinFWD, GPIO.OUT)
        GPIO.setup(self._pinREV, GPIO.OUT)
        GPIO.output(self._pinFWD, GPIO.LOW)
        GPIO.output(self._pinREV, GPIO.LOW)
        self._PWM = GPIO.PWM(self._pinPWM, MotorControl.PWM_FREQ)
        self._PWM.start(MotorControl.PWM_DEFAULT_SPD)

    def SetVoltageLookupTable(self, table):
        self._lookupTable = table

    def SetPwmDutyCycle(self, pwm):
        if pwm < 0.0:
            pwm = 0.0
        if pwm > 100.0:
            pwm = 100.0
        
        self._PWM.ChangeDutyCycle(pwm)

    def SetVoltage(self, V):
        if V > 0.0:
            GPIO.output(self._pinFWD, GPIO.HIGH)
            GPIO.output(self._pinREV, GPIO.LOW)
        if V < 0.0:
            GPIO.output(self._pinFWD, GPIO.LOW)
            GPIO.output(self._pinREV, GPIO.HIGH)
        self._PWM.ChangeDutyCycle(self._lookupTable.Lookup(abs(V)))

    def Brake(self):
        GPIO.output(self._pinFWD, GPIO.LOW)
        GPIO.output(self._pinREV, GPIO.LOW)

    def Float(self):
        GPIO.output(self._pinFWD, GPIO.HIGH)
        GPIO.output(self._pinREV, GPIO.HIGH)