import math
import numpy as np
g = 9.81
class ControlLogic:
    def calcAzimuthDelta(self, unsteady_parafoil_state, target, deltas):
        targ = [target[0] - unsteady_parafoil_state['X-position'], target[1] - unsteady_parafoil_state['Y-position'], 0]
        vehicle = [math.cos(math.radians(unsteady_parafoil_state['Azimuth Angle'])), math.sin(math.radians(unsteady_parafoil_state['Azimuth Angle'])), 0]
        mag = math.sqrt(targ[0] ** 2 + targ[1] ** 2)
        dot = np.dot(vehicle, targ)
        cross = np.cross(vehicle, targ)
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

    def convertBankAngleToDeflect(self, parafoil, unsteady_parafoil_state):
        turn_rate = unsteady_parafoil_state['Bank Angle'] * g / unsteady_parafoil_state['Velocity']
        deflect_angle = math.degrees(turn_rate / 0.625 / unsteady_parafoil_state['Velocity'] * parafoil['Span'])

     # Blame max ross for this mess
    def convertServoToDeflect(self, parafoil, servo_angle):
        spool_d = 0.5                                       # in Spool Diameter
        spool_per = 2 * math.pi * spool_d                   # in Spool Perimeter​ 
        line_delta = spool_per / (360 / servo_angle)        # Maximum Spool Length
        chord = parafoil['Chord'] * 39.37                   # in Parafoil Width
        height = parafoil['Line']['Length'] * 39.37         # in Height of Parafoil from Spool
        l_0 = math.sqrt(height ** 2 + (chord / 2) ** 2)
        l_min = l_0 - line_delta
        hypot = math.sqrt(height ** 2 + (chord / 4) ** 2)                                               # Shared Side of Triangles
        theta1 = math.atan(height / (chord / 4))                                                        # Angle of the Fixed Triangle
        theta2 = math.acos(((chord / 4) ** 2 + hypot ** 2 - l_min ** 2) / (2 * (chord / 4) * hypot))    # Angle of the Deflected Triangle
        deflection = math.pi - math.degrees(theta1) - math.degrees(theta2)                                                          # Angle of Deflection
        return deflection

    # Blame max ross for this mess
    def convertDeflectToServo(self, parafoil, deflection):
        spool_d = 0.5                                       # in Spool Diameter
        spool_per = 2 * math.pi * spool_d                   # in Spool Perimeter​ 
        chord = parafoil['Chord'] * 39.37                   # in Parafoil Width
        height = parafoil['Line']['Length'] * 39.37         # in Height of Parafoil from Spool
        theta_1 = math.degrees(math.atan(height / (chord / 4)))            # Angle of the Fixed Triangle
        theta_2 = math.pi - deflection - theta_1                # Angle of the Deflected Triangle
        hypot = math.sqrt(height ** 2 + (chord / 4) ** 2)                                               # Shared Side of Triangles
        length = math.sqrt((chord / 4) ** 2 + hypot ** 2 - 2 * chord / 4 * hypot * math.cos(math.radians(theta_2)))
        length_0 = math.sqrt(height ** 2 + (chord / 2) ** 2)
        line_delta = length_0 - length
        servo_angle = 360 / (spool_per / line_delta)
        return servo_angle
parafoil = {
    'Chord' : 0.381,
    'Line' : {
        'Length' : 0.457
    }
}
ctrl = ControlLogic()
servo = ctrl.convertDeflectToServo(parafoil, 5)
print(servo)
deflect = ctrl.convertServoToDeflect(parafoil, 10)
print(deflect)