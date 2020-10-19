import math
import numpy as np
from util import util
g = 9.81
class ControlLogic:
    # def proportionalController(self, kp, sp):
    #     mv = 0
    #     while True:
    #         pv = yield mv
    #         mv = kp * (sp - pv)

    # Calculates delta angle between payload heading and target
    def calcAzimuthDelta(self, unsteady_parafoil_state, target, deltas):
        targ = [target[0] - unsteady_parafoil_state['X-position'], target[1] - unsteady_parafoil_state['Y-position'], 0]
        vehicle = [math.cos(math.radians(unsteady_parafoil_state['Azimuth Angle'])), math.sin(math.radians(unsteady_parafoil_state['Azimuth Angle'])), 0]
        mag = math.sqrt(targ[0] ** 2 + targ[1] ** 2)
        dot = np.dot(vehicle, targ)
        cross = np.cross(vehicle, targ)
        dot = util.clamp(dot, abs(mag), -abs(mag))
        delta = math.degrees(math.acos(dot / mag))
        if cross[2] < 0:
            deltas.append(-1 * delta)
            return -1 * delta
        elif cross[2] > 0:
            deltas.append(delta)
            return delta
    
    # def convertDeflectToBankAngle(self, parafoil, parafoil_state, deflect_angle):
    #     turn_rate = 0.625 * parafoil_state['Velocity'] / parafoil["Span"] * math.radians(deflect_angle)
    #     return parafoil_state['Velocity'] * turn_rate / g

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