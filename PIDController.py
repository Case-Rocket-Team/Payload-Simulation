
class PIDController:

    lastOffset = 0
    accum = 0
    
    def __init__(self, kP, kI, kD, setpoint):
        print("controller initialized w/ kp:" + str(kP) + ", KD:" + str(kD) + "ki"+str(kI))
        self.kP, self.kI, self.kD = kP, kI, kD
        self.setpoint = setpoint
        self.max, self.min = max, min

    def calculate(self, state, dt):
        offset = state - self.setpoint
        proportional = self.kP * offset
        derivative = self.kD * (offset - self.lastOffset) / dt
        self.accum += offset*dt
        integral = self.kI * self.accum
        #self.deadband(integral, self.max, self.min)
        self.lastOffset = offset
        return proportional + derivative + integral

    @staticmethod
    def deadband(input, max, min):
        if(input > max):
            return min
        if(input < min):
            return min
        return input