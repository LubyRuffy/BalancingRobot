class Filter:
    def __init__(self, timeConst, valInit=0.0):
        """Initializes the first fiter. The filter is of the form:
        output = (1 - a) * output_old + a * input

        where:
        a = dt / (timeConst + dt)

        Aguments:
        timeConst -- Filter time constant (s)
        valInit -- Initial output of the filter
        """

        self._valOut = valInit
        self._timeConst = float(timeConst)

    def Filter(self, dt, input):
        """Filters an input value.

        Arguments:
        dt - Time since last call (s)
        input - Input value

        Returns:
        Filtered value.
        """

        a = 1
        if dt > 1e-8:
            a = dt / (self._timeConst + dt)
        self._valOut = (1 - a) * self._valOut + a * input
        return self._valOut

    def Value(self):
        """Returns the current value of the filter.
        """
        return self._valOut

    def Reset(self, valInit=0.0):
        """Resets the filter.

        Arguments:
        valInit -- The value to set the filter output to.
        """

        self._valOut = valInit
