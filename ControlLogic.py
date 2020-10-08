import math
g = 9.81
class ControlLogic:
    def proportionalController(self, kp, sp):
        mv = 0
        while True:
            pv = yield mv
            mv = kp * (sp - pv)

    # Returns angle of target off of x-axis in degrees
    def calcTargAngle(self, target):
        if target[0] != 0:
            targ_angle = math.degrees(math.atan(target[1] / target[0]))
        elif target[0] == 0:
            if target[1] > 0:
                targ_angle = 0
            elif target[1] < 0:
                targ_angle = 180
        if targ_angle < 0:
            targ_angle += 360
        return targ_angle

    # Calculates delta angle between payload heading and target angle
    def calcAzimuthDelta(self, unsteady_parafoil_state, targ_angle, deltas):
        delta = math.radians(unsteady_parafoil_state['Azimuth Angle'] - targ_angle)
        deltas.append(math.degrees(delta))
        return delta
    
    def convertDeflectToBankAngle(self, parafoil, parafoil_state, deflect_angle):
        turn_rate = 0.625 * parafoil_state['Velocity'] / parafoil["Span"] * deflect_angle 
        return parafoil_state['Velocity'] * turn_rate / g

    # def convertDeflectToServo(self):
    #     spool_d = 0.5 #in Spool Diameter
    #     spool_per = 2*math.pi*spool_d #in Spool Perimeter​
    #     maxSigma = 100 # Maximum Servo Angle
    #     maxDL = spool_per/(360/maxSigma) # Maximum Spool Length
    #     W = 15 #in Parafoil Width
    #     h = 18 #in Height of Parafoil from Spool
    #     L0 = math.sqrt(h**2 + (W/2)**2)
    #     Lmin = L0 - maxDL
    #     hypot = math.sqrt(h**2 + (W/4)**2) #Shared Side of Triangles
    #     theta1 = math.atan(h/(W/4)) # Angle of the Fixed Triangle
    #     theta2 = math.acos(((W/4)**2+hypot**2-Lmin**2)/(2*(W/4)*hypot)) #Angle of the Deflected Triangle
    #     theta = math.pi-theta1-theta2 #Total Angle of Deflection
    #     return