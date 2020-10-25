import math
import numpy as np
from util import util
g = 9.81
class ControlCalculations:
    # Calculates the angle delta between the heading of the vehicle and the target distance vector
    # Also assigns +/- to the angle with positive (+) left and negative (-) right
    def calcAzimuthDelta(self, unsteady_parafoil_state, target, deltas):
        targ = [target[0] - unsteady_parafoil_state['X-position'], target[1] - unsteady_parafoil_state['Y-position'], 0]
        vehicle = [math.cos(math.radians(unsteady_parafoil_state['Azimuth Angle'])), math.sin(math.radians(unsteady_parafoil_state['Azimuth Angle'])), 0]
        mag = math.sqrt(targ[0] ** 2 + targ[1] ** 2)
        dot = np.dot(vehicle, targ)
        # Use cross product to determine which side the target is on
        cross = np.cross(vehicle, targ)
        # Keep rounding errors from leading to dot product errors within the arccosine
        dot = util.clamp(dot, abs(mag), -abs(mag))
        delta = math.degrees(math.acos(dot / mag))
        if cross[2] >= 0:
            deltas.append(delta)
            return delta
        elif cross[2] < 0:
            deltas.append(-1 * delta)
            return -1 * delta
        
    # Calculates the magnitude of the distance vector between the vehicle and the target
    def calcTargMag(self, unsteady_parafoil_state, target):
        targ = [target[0] - unsteady_parafoil_state['X-position'], target[1] - unsteady_parafoil_state['Y-position'], 0]
        mag = math.sqrt(targ[0] ** 2 + targ[1] ** 2)
        return mag

    # Converts bank angle of vehicle into deflection angle of wing
    def convertBankAngleToDeflect(self, parafoil, unsteady_parafoil_state):
        turn_rate = math.sin(math.radians(unsteady_parafoil_state['Bank Angle'])) * g / (unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])))
        deflect_angle = math.degrees(turn_rate / 0.625 / unsteady_parafoil_state['Velocity'] * parafoil['Span'])
        return deflect_angle

    # Blame max ross for this mess
    # Probably converts deflection angle of wing into a servo angle
    def convertDeflectToServo(self, parafoil, deflection):
        # Spool radius in m
        spool_r = 0.0127
        # Spool circumference in m
        spool_cir = 2 * math.pi * spool_r
        # Angle of the Fixed Triangle
        theta_1 = math.atan(parafoil['Line']['Length'] / (parafoil['Chord'] / 4))
        # Angle of the Deflected Triangle
        theta_2 = math.pi - math.radians(deflection) - theta_1
        # Shared Side of Triangles
        hypot = math.sqrt(parafoil['Line']['Length'] ** 2 + (parafoil['Chord'] / 4) ** 2)
        length = math.sqrt((parafoil['Chord'] / 4) ** 2 + hypot ** 2 - (2 * parafoil['Chord'] / 4 * hypot * math.cos(theta_2)))
        length_0 = math.sqrt(parafoil['Line']['Length'] ** 2 + (parafoil['Chord'] / 2) ** 2)
        line_delta = length_0 - length
        servo_angle = 360 / (spool_cir / line_delta)
        return servo_angle

    # !!! might need fixed to have variable indicate whether left or right starting in bank angle to deflect conversion
    def servoUpdater(self, left_servo_angle, right_servo_angle, servo_angle):
        if servo_angle > 0:
            right_servo_angle = 0
            left_servo_angle = servo_angle
        elif servo_angle < 0:
            left_servo_angle = 0
            right_servo_angle = abs(servo_angle)
        else:
            left_servo_angle = 0
            right_servo_angle = 0
        return left_servo_angle, right_servo_angle

