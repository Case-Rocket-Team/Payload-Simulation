from util import util
class PIDController:

    lastOffset = None
    accum = 0
    
    def __init__(self, kP, kI, kD, setpoint):
        # print("controller initialized w/ kp:" + str(kP) + ", KD:" + str(kD) + "ki"+str(kI))
        self.kP, self.kI, self.kD = kP, kI, kD
        self.setpoint = setpoint
        self.max, self.min = max, min
        self.max, self.min = 0, 0

    def setIntegralLimits(self, maxx, minn):
        self.max = maxx
        self.min = minn

    def calculate(self, state, dt):
        offset = state - self.setpoint
        if self.lastOffset == None:
            self.lastOffset = offset
        proportional = self.kP * offset
        derivative = self.kD * (offset - self.lastOffset) / dt
        self.accum += offset * dt
        integral = util.clamp(self.kI * self.accum, self.max, self.min)
        self.lastOffset = offset
        return (proportional + derivative + integral), proportional, integral, derivative
