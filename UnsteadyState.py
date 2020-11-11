import math
import numpy as np
import copy
from ControlCalculations import ControlCalculations
from PIDController import PIDController
from util import util 

class UnsteadyState:
    def __init__(self):
        self.density = 1.225
        self.g = 9.81
    # Calculate total velocity magnitude
    def calcVelocity(self, parafoil, unsteady_parafoil_state, lift_force, drag_force, dt):
        weight = (parafoil['Payload Mass'] + parafoil['Canopy Mass']) * self.g
        acceleration = -((drag_force + weight * math.sin(math.radians(unsteady_parafoil_state['Glide Angle']))) / (parafoil['Payload Mass'] + parafoil['Canopy Mass']))
        unsteady_parafoil_state['Velocity'] += acceleration * dt

    airConditions = {
        #constants
        'MolarMass': 0.0289647,
        'R': 8.31446261815324,
        'T': 21+273.15,
        'po': 1.225,
        #variables
        'p': 0
    }

    # Zach's air density updater, could be important for higher altitude flights
    def getAirDensity(self, unsteady_parafoil_state):
        M = self.airConditions['MolarMass']
        R = self.airConditions['R']
        T = self.airConditions['T']
        po = self.airConditions['po']

        self.density = po * math.exp((-9.8 * M * unsteady_parafoil_state['Altitude']) / (R * T))

    # Calculate Lift Force
    def calcLiftForce(self, parafoil, unsteady_parafoil_state):
        return 0.5 * self.density * (unsteady_parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CL']
    
    # Calculate Drag Force
    def calcDragForce(self, parafoil, unsteady_parafoil_state):
        return 0.5 * self.density * (unsteady_parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CD']
    
    # Calculate Glide Angle
    def calcGlideAngle(self, parafoil, unsteady_parafoil_state, lift_force, drag_force, dt):
        weight = (parafoil['Payload Mass'] + parafoil['Canopy Mass']) * self.g
        roc_glide_angle = (lift_force * math.cos(math.radians(unsteady_parafoil_state['Bank Angle'])) - (weight * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])))) / ((parafoil['Payload Mass'] + parafoil['Canopy Mass']) * unsteady_parafoil_state['Velocity'])
        unsteady_parafoil_state['Glide Angle'] += roc_glide_angle * dt

    # Calculate Turning Velocity
    def calcTurningVelocity(self, parafoil, unsteady_parafoil_state, lift_force):
        return math.degrees((lift_force * math.sin(math.radians(unsteady_parafoil_state['Bank Angle']))) / ((parafoil['Payload Mass'] + parafoil['Canopy Mass']) * unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle']))))
    
    # Fixes azimuth angle to remain positive and under 360 deg
    def fixAzimuth(self, unsteady_parafoil_state):
        if unsteady_parafoil_state['Azimuth Angle'] > 360 or unsteady_parafoil_state['Azimuth Angle'] < 0:
            if unsteady_parafoil_state['Azimuth Angle'] > 360:
                unsteady_parafoil_state['Azimuth Angle'] -= 360
            elif unsteady_parafoil_state['Azimuth Angle'] < 0:
                unsteady_parafoil_state['Azimuth Angle'] += 360

    # Kinematic update method
    def updatePosition(self, parafoil, unsteady_parafoil_state, dt):
        self.getAirDensity(unsteady_parafoil_state)
        lift_force = self.calcLiftForce(parafoil, unsteady_parafoil_state)
        drag_force = self.calcDragForce(parafoil, unsteady_parafoil_state)
        self.calcGlideAngle(parafoil, unsteady_parafoil_state, lift_force, drag_force, dt)
        self.calcVelocity(parafoil, unsteady_parafoil_state, lift_force, drag_force, dt)
        # Update x,y,z position of the vehicle based on velocities
        x_velocity = unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])) * math.cos(math.radians(unsteady_parafoil_state['Azimuth Angle'])) + unsteady_parafoil_state['Wind Field'][0]
        unsteady_parafoil_state['X-position'] += x_velocity * dt
        y_velocity = unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])) * math.sin(math.radians(unsteady_parafoil_state['Azimuth Angle'])) + unsteady_parafoil_state['Wind Field'][1]
        unsteady_parafoil_state['Y-position'] += y_velocity * dt
        z_velocity = unsteady_parafoil_state['Velocity'] * math.sin(math.radians(unsteady_parafoil_state['Glide Angle'])) + unsteady_parafoil_state['Wind Field'][2]
        unsteady_parafoil_state['Altitude'] += z_velocity * dt
        # Update heading
        turning_rate =  self.calcTurningVelocity(parafoil, unsteady_parafoil_state, lift_force)
        unsteady_parafoil_state['Azimuth Angle'] += turning_rate * dt
        self.fixAzimuth(unsteady_parafoil_state)
        # Prints vehicle info on final iteration
        # if unsteady_parafoil_state['Altitude'] < 0:
        #     print("Downwind (x) Velocity: ", x_velocity, "m/s, Crosswind (y) Velocity: ", y_velocity, "m/s, Vertical Velocity: ", z_velocity, "m/s, Overall Velocity: ", unsteady_parafoil_state['Velocity'], "m/s, Glide Angle", unsteady_parafoil_state['Glide Angle'], "degrees, Azimuth Angle: ", unsteady_parafoil_state['Azimuth Angle'], "degrees, Turning Rate: ", turning_rate, "degrees/s")

    # Straight approach control loop
    def runStraightControlLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, kp, kd, ki, dt):
        print(kp)
        ctrl = ControlCalculations()
        unsteady_parafoil_state['Velocity'] = parafoil_state['Velocity']
        unsteady_parafoil_state['Glide Angle'] = parafoil_state['Glide Angle']
        unsteady_times = [0]
        unsteady_angles = [0]
        unsteady_x_positions = [unsteady_parafoil_state['X-position']]
        unsteady_y_positions = [unsteady_parafoil_state['Y-position']]
        unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
        unsteady_azimuths = [unsteady_parafoil_state['Azimuth Angle']]
        unsteady_bank_angles = [unsteady_parafoil_state['Bank Angle']]
        left_servo_angle, right_servo_angle = 0, 0
        left_servo_angles, right_servo_angles = [left_servo_angle], [right_servo_angle]
        deflections = [0]
        unsteady_time = 0
        deltas = []
        ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
        # Set up P controller with the process variable being the delta angle between the azimuth heading the the target, the manipulated variable being the assymetric flap deflection
        pid = PIDController(kp, kd, ki, 0)
        # Run loop that iterates the kinematic unsteady state equations
        while unsteady_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            delta = ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
            bank_angle = pid.calculate(delta, dt)
            unsteady_parafoil_state['Bank Angle'] = util.clamp(bank_angle, 25, -25)
            deflection_angle = ctrl.convertBankAngleToDeflect(parafoil, unsteady_parafoil_state)
            servo_angle = ctrl.convertDeflectToServo(parafoil, deflection_angle)
            left_servo_angle, right_servo_angle = ctrl.servoUpdater(left_servo_angle, right_servo_angle, servo_angle)
            left_servo_angles.append(left_servo_angle)
            right_servo_angles.append(right_servo_angle)
            deflections.append(deflection_angle)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            unsteady_bank_angles.append(unsteady_parafoil_state['Bank Angle'])
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
        # print("^^^Time is: ", unsteady_time, "Position is (x,y,z): (", unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'], ")")
        return unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections

    # Dist to Target control loop
    def runDistToTargControlLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, kp, ki, kd, dt):
        ctrl = ControlCalculations()
        unsteady_parafoil_state['Velocity'] = parafoil_state['Velocity']
        unsteady_parafoil_state['Glide Angle'] = parafoil_state['Glide Angle']
        unsteady_times = [0]
        unsteady_angles = [0]
        unsteady_x_positions = [unsteady_parafoil_state['X-position']]
        unsteady_y_positions = [unsteady_parafoil_state['Y-position']]
        unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
        unsteady_azimuths = [unsteady_parafoil_state['Azimuth Angle']]
        unsteady_bank_angles = [unsteady_parafoil_state['Bank Angle']]
        unsteady_mags = [ctrl.calcTargMag(unsteady_parafoil_state, target)]
        unsteady_time = 0
        pid = PIDController(kp, ki, kd, 150)
        pid.setIntegralLimits(25, -25)
        count = 1
        while unsteady_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            targDist = ctrl.calcTargMag(unsteady_parafoil_state, target)
            bank_angle = pid.calculate(targDist, dt)
            unsteady_parafoil_state['Bank Angle'] = util.clamp(bank_angle, 25, -25)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            unsteady_bank_angles.append(unsteady_parafoil_state['Bank Angle'])
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
            unsteady_mags.append(targDist)
        print("Kp: ", kp, "Distance to Target: ", ctrl.calcTargMag(unsteady_parafoil_state, target), " m")
        return unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, unsteady_azimuths, unsteady_bank_angles, unsteady_mags

    # Simulated straight approach control loop
    def runSimStraightControlLoop(self, parafoil, unsteady_parafoil_state, target, kp, kd, ki, dt):
        ctrl = ControlCalculations()
        deltas = []
        sim_parafoil_state = copy.deepcopy(unsteady_parafoil_state)
        ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
        # Set up P controller with the process variable being the delta angle between the azimuth heading the the target, the manipulated variable being the assymetric flap deflection
        pid = PIDController(kp, kd, ki, 0)
        # Run loop that iterates the kinematic unsteady state equations
        while sim_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, sim_parafoil_state, dt)
            delta = ctrl.calcAzimuthDelta(sim_parafoil_state, target, deltas)
            bank_angle = pid.calculate(delta, dt)
            sim_parafoil_state['Bank Angle'] = util.clamp(bank_angle, 30, -30)
        if math.sqrt(math.pow(sim_parafoil_state['X-position'] - target[0], 2) + math.pow(sim_parafoil_state['Y-position'] - target[1], 2)) <= 10:
            return False
        else:
            return True

    # Return to Pad control loop
    def runRTPControlLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, kp1, kp2, ki1, ki2, kd1, kd2, dt):
        ctrl = ControlCalculations()
        unsteady_parafoil_state['Velocity'] = parafoil_state['Velocity']
        unsteady_parafoil_state['Glide Angle'] = parafoil_state['Glide Angle']
        unsteady_times = [0]
        unsteady_angles = [0]
        unsteady_x_positions = [unsteady_parafoil_state['X-position']]
        unsteady_y_positions = [unsteady_parafoil_state['Y-position']]
        unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
        unsteady_azimuths = [unsteady_parafoil_state['Azimuth Angle']]
        unsteady_bank_angles = [unsteady_parafoil_state['Bank Angle']]
        proportionals = [0]
        integrals = [0]
        derivatives = [0]
        unsteady_mags = [ctrl.calcTargMag(unsteady_parafoil_state, target)]
        left_servo_angle, right_servo_angle = 0, 0
        left_servo_angles, right_servo_angles = [left_servo_angle], [right_servo_angle]
        deflections = [0]
        unsteady_time = 0
        deltas = []
        # Run check to see what region target is in
        delta_prime = ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
        # Set up P controller with the process variable being the delta angle between the azimuth heading the the target, the manipulated variable being the vehicle bank angle
        # Attempting to keep the vehicle circling the pad, so 90 deg angle b/w target dist vector and heading
        loop_check = True
        call = 0
        pid = PIDController(kp1, ki1, kd1, 150)
        pid.setIntegralLimits(200, -200)
        count = 0
        sim_count = 0
        if delta_prime >= 0:
            w = 1
        else:
            w = -1
        # Run loop that iterates the kinematic unsteady state equations
        # First loop attempts to keep payload in a circle around the pad
        while loop_check and unsteady_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            targDist = ctrl.calcTargMag(unsteady_parafoil_state, target)
            bank_angle, proportional, integral, derivative = pid.calculate(targDist, dt)
            unsteady_parafoil_state['Bank Angle'] = util.clamp(w * bank_angle, 25, -25)
            deflection_angle = ctrl.convertBankAngleToDeflect(parafoil, unsteady_parafoil_state)
            servo_angle = ctrl.convertDeflectToServo(parafoil, deflection_angle)
            left_servo_angle, right_servo_angle = ctrl.servoUpdater(left_servo_angle, right_servo_angle, servo_angle)
            left_servo_angles.append(left_servo_angle)
            right_servo_angles.append(right_servo_angle)
            deflections.append(deflection_angle)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            proportionals.append(proportional)
            integrals.append(integral)
            derivatives.append(derivative)
            unsteady_bank_angles.append(unsteady_parafoil_state['Bank Angle'])
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
            unsteady_mags.append(targDist)
            '''
            # After reaching a certain height, start looking for approach path that will arrive <10m for target
            if unsteady_parafoil_state['Altitude'] < 200:
                # print("Sim run ", sim_count)
                if call == 0:
                    print("Altitude: " , unsteady_parafoil_state['Altitude'], "Distance to target", ctrl.calcTargMag(unsteady_parafoil_state, target))
                    call += 1
                loop_check = self.runSimStraightControlLoop(parafoil, unsteady_parafoil_state, target, kp2, ki2, kd2, dt)
                sim_count += 1
            if unsteady_parafoil_state['Altitude'] < 60:
                loop_check = False
            '''
            count += 1
        count_terminator = count
        print("Approach start altitude: ", unsteady_parafoil_state['Altitude'])
        # Set up P controller with the process variable being the delta angle between the azimuth heading the the target, the manipulated variable being the vehicle bank angle
        pid2 = PIDController(kp2, ki2, kd2, 0)
        # Run loop that iterates the kinematic unsteady state equations
        # Second Loop turns out of the circle and approaches target
        while unsteady_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            delta = ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
            bank_angle = pid2.calculate(delta, dt)
            unsteady_parafoil_state['Bank Angle'] = util.clamp(bank_angle, 30, -30)
            deflection_angle = ctrl.convertBankAngleToDeflect(parafoil, unsteady_parafoil_state)
            servo_angle = ctrl.convertDeflectToServo(parafoil, deflection_angle)
            left_servo_angle, right_servo_angle = ctrl.servoUpdater(left_servo_angle, right_servo_angle, servo_angle)
            left_servo_angles.append(left_servo_angle)
            right_servo_angles.append(right_servo_angle)
            deflections.append(deflection_angle)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            unsteady_bank_angles.append(unsteady_parafoil_state['Bank Angle'])
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
            count += 1
        print("^^^Time is: ", unsteady_time, "Position is (x,y,z): (", unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'], ")")
        return unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_mags, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections, count_terminator, count, proportionals, integrals, derivatives

    # Sim run without controls
    def runLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, dt):
        ctrl = ControlCalculations()
        unsteady_parafoil_state['Velocity'] = parafoil_state['Velocity']
        unsteady_parafoil_state['Glide Angle'] = parafoil_state['Glide Angle']
        unsteady_times = [0]
        unsteady_angles = [0]
        unsteady_x_positions = [unsteady_parafoil_state['X-position']]
        unsteady_y_positions = [unsteady_parafoil_state['Y-position']]
        unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
        unsteady_azimuths = [unsteady_parafoil_state['Azimuth Angle']]
        unsteady_time = 0
        deltas = []
        ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
        while unsteady_parafoil_state['Altitude'] > 0:
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
        print("^^^Time is: ", unsteady_time, "Position is (x,y,z): (", unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'], ")")
        return unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths