#!/usr/bin/env python

import pigpio

class RotaryEncoder:
    """Class to decode mechanical rotary encoder pulses."""

    def __init__(self, pi, gpioA, gpioB):
        """
        Instantiate the class with the pi and gpios connected to
        rotary encoder contacts A and B.  The common contact
        should be connected to ground. It takes
        one parameter which is +1 for clockwise and -1 for
        counterclockwise.

        EXAMPLE

        import time
        import pigpio
        import RotaryEncoder

        pi = pigpio.pi()
        decoder = RotaryEncoder.RotaryEncoder(pi, 7, 8)

        time.sleep(300)

        decoder.cancel()

        pi.stop()
        """

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.count = 0

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        """
        Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
        """

        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level;

        if gpio != self.lastGpio: # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.count = self.count + 1
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.count = self.count - 1

    def cancel(self):
        """
        Cancel the rotary encoder decoder.
        """

        self.cbA.cancel()
        self.cbB.cancel()

    def getCount(self):
        """
        Returns the current encoder count.
        """

        return self.count

if __name__ == "__main__":

   import time
   import pigpio

   from RotaryEncoder import *

   pi = pigpio.pi()

   decoder = RotaryEncoder(pi, 19, 26)
   prev_time = time.time()
   prev_count = 0

   while 1:
      current_time = time.time()
      dt = current_time - prev_time
      prev_time = current_time
      current_count = decoder.getCount()
      dcount = current_count - prev_count
      prev_count = current_count
      rpm = (1/dt) * (float(dcount) / (16.0*90.0))*60.0
      print "RPM: {0}".format(rpm)
      time.sleep(0.1)

   decoder.cancel()
   pi.stop()
