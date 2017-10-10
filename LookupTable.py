class LookupTable:
    def __init__(self, x, y):
        assert len(x) == len(y)
        assert len(x) > 1
        assert len(y) > 1
        self._x = x
        self._y = y
        assert self.CheckMonotonic()

    def CheckMonotonic(self):
        lastX = self._x[0]
        for x in self._x:
            if x < lastX:
                return False
            lastX = x
        return True

    def Lookup(self, x):
        y = 0.0
        if x <= self._x[0]:
            return self._y[0]
        if x >= self._x[-1]:
            return self._y[-1]
        for idx, elem in enumerate(self._x):
            if x >= elem:
                ratio = float(x - elem) / float(self._x[idx+1] - self._x[idx])
                y = self._y[idx] + ratio*(self._y[idx+1] - self._y[idx])
            if x < elem:
                break
        return y
