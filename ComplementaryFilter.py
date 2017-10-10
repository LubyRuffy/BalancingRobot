class ComplementaryFilter:
    def __init__(self, alpha, valInit):
        """Initializes the fiter. The filter is of the form:
        output = (1 - alpha) * (output_old * gyro * dt) + alpha * acc

        Aguments:
        alpha -- Filter constant
        valInit -- Initial output of the filter
        """

        self._valOut = valInit
        self._alpha = alpha

    def Filter(self, dt, gyroData, accData):
        self._valOut = (1 - self._alpha) * (self._valOut + gyroData*dt) \
            + (self._alpha * accData)
        return self._valOut

    def Reset(self, valInit):
        """Resets the filter.

        Arguments:
        valInit -- The value to set the filter output to.
        """

        self._valOut = valInit
