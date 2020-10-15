import math
from ControlLogic import ControlLogic
from simple_pid import PID
g = 9.81
density = 1.225
class UnsteadyState:
    # Calculate total velocity magnitude
    def calcVelocity(self, parafoil, unsteady_parafoil_state, lift_force, drag_force, dt):
        weight = (parafoil['Payload Mass'] + parafoil['Canopy Mass']) * g
        acceleration = -((drag_force + weight * math.sin(math.radians(unsteady_parafoil_state['Glide Angle']))) / (parafoil['Payload Mass'] + parafoil['Canopy Mass']))
        unsteady_parafoil_state['Velocity'] += acceleration * dt
    
    # Calculate Lift Force
    def calcLiftForce(self, parafoil, unsteady_parafoil_state):
        return 0.5 * density * (unsteady_parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CL']
    
    # Calculate Drag Force
    def calcDragForce(self, parafoil, unsteady_parafoil_state):
        return 0.5 * density * (unsteady_parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CD']
    
    # Calculate Glide Angle
    def calcGlideAngle(self, parafoil, unsteady_parafoil_state, lift_force, drag_force, dt):
        weight = (parafoil['Payload Mass'] + parafoil['Canopy Mass']) * g
        roc_glide_angle = (lift_force * math.cos(math.radians(unsteady_parafoil_state['Bank Angle'])) - (weight * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])))) / ((parafoil['Payload Mass'] + parafoil['Canopy Mass']) * unsteady_parafoil_state['Velocity'])
        unsteady_parafoil_state['Glide Angle'] += roc_glide_angle * dt

    # Calculate Turning Velocity
    def calcTurningVelocity(self, parafoil, unsteady_parafoil_state, lift_force):
        return math.degrees((lift_force * math.sin(math.radians(unsteady_parafoil_state['Bank Angle']))) / ((parafoil['Payload Mass'] + parafoil['Canopy Mass']) * unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle']))))
    
    # Run loop
    def updatePosition(self, parafoil, unsteady_parafoil_state, dt):
        lift_force = self.calcLiftForce(parafoil, unsteady_parafoil_state)
        drag_force = self.calcDragForce(parafoil, unsteady_parafoil_state)
        self.calcGlideAngle(parafoil, unsteady_parafoil_state, lift_force, drag_force, dt)
        self.calcVelocity(parafoil, unsteady_parafoil_state, lift_force, drag_force, dt)
        # Update x,y,z position of the vehicle based on velocities
        x_velocity = unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])) * math.cos(math.radians(unsteady_parafoil_state['Azimuth Angle']))
        unsteady_parafoil_state['X-position'] += x_velocity * dt
        y_velocity = unsteady_parafoil_state['Velocity'] * math.cos(math.radians(unsteady_parafoil_state['Glide Angle'])) * math.sin(math.radians(unsteady_parafoil_state['Azimuth Angle']))
        unsteady_parafoil_state['Y-position'] += y_velocity * dt
        z_velocity = unsteady_parafoil_state['Velocity'] * math.sin(math.radians(unsteady_parafoil_state['Glide Angle']))
        unsteady_parafoil_state['Altitude'] += z_velocity * dt
        # Update heading
        turning_rate =  self.calcTurningVelocity(parafoil, unsteady_parafoil_state, lift_force)
        unsteady_parafoil_state['Azimuth Angle'] += turning_rate * dt
        if unsteady_parafoil_state['Azimuth Angle'] > 360 or unsteady_parafoil_state['Azimuth Angle'] < 0:
            if unsteady_parafoil_state['Azimuth Angle'] > 360:
                unsteady_parafoil_state['Azimuth Angle'] -= 360
            elif unsteady_parafoil_state['Azimuth Angle'] < 0:
                unsteady_parafoil_state['Azimuth Angle'] += 360

        # if unsteady_parafoil_state['Azimuth Angle'] > 180:
        #     unsteady_parafoil_state['Azimuth Angle'] -= 360
        if unsteady_parafoil_state['Altitude'] < 0:
            print("Downwind (x) Velocity: ", x_velocity, "m/s, Crosswind (y) Velocity: ", y_velocity, "m/s, Vertical Velocity: ", z_velocity, "m/s, Overall Velocity: ", unsteady_parafoil_state['Velocity'], "m/s, Glide Angle", unsteady_parafoil_state['Glide Angle'], "degrees, Azimuth Angle: ", unsteady_parafoil_state['Azimuth Angle'], "degrees, Turning Rate: ", turning_rate, "degrees/s")

    def checkBankAngle(self, unsteady_parafoil_state):
        if unsteady_parafoil_state['Bank Angle'] > 20:
            unsteady_parafoil_state['Bank Angle'] = 20
        elif unsteady_parafoil_state['Bank Angle'] < -20:
            unsteady_parafoil_state['Bank Angle'] = -20

    def runControlLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, kp, dt):
        print(kp)
        ctrl = ControlLogic()
        unsteady_parafoil_state['Velocity'] = parafoil_state['Velocity']
        unsteady_parafoil_state['Glide Angle'] = parafoil_state['Glide Angle']
        unsteady_times = [0]
        unsteady_angles = [0]
        unsteady_x_positions = [unsteady_parafoil_state['X-position']]
        unsteady_y_positions = [unsteady_parafoil_state['Y-position']]
        unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
        unsteady_azimuths = [unsteady_parafoil_state['Azimuth Angle']]
        unsteady_bank_angles = [unsteady_parafoil_state['Bank Angle']]
        unsteady_time = 0
        deltas = []
        # Run check to see what region target is in
        ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
        # Set up P controller with the process variable being the delta angle between the azimuth heading the the target, the manipulated variable being the assymetric flap deflection
        # pcontroller = ctrl.proportionalController(kp, 0)
        # pcontroller.send(None)
        pid = PID(kp, 0, 0, setpoint=0)
        #pid.sample_time = 0.25
        pid.output_limits = (-20, 20)
        # Run loop that iterates the kinematic unsteady state equations
        while unsteady_parafoil_state['Altitude'] > 0:
            #print("Time is: ", unsteady_time)
            self.updatePosition(parafoil, unsteady_parafoil_state, dt)
            delta = ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas)
            # unsteady_parafoil_state['Bank Angle'] = pcontroller.send(delta)
            unsteady_parafoil_state['Bank Angle'] = -1 * pid(delta)
            # self.checkBankAngle(unsteady_parafoil_state)
            # deflection = pcontroller.send(ctrl.calcAzimuthDelta(unsteady_parafoil_state, target, deltas))
            # unsteady_parafoil_state['Bank Angle'] = ctrl.convertDeflectToBankAngle(parafoil, unsteady_parafoil_state, deflection)
            unsteady_time += dt
            unsteady_times.append(unsteady_time)
            unsteady_bank_angles.append(unsteady_parafoil_state['Bank Angle'])
            unsteady_azimuths.append(unsteady_parafoil_state['Azimuth Angle'])
            unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
            unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
            unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
            unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
            #print(unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'])
        # pcontroller.close()
        print("^^^Time is: ", unsteady_time, "Position is (x,y,z): (", unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'], ")")
        return unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles

    def runLoop(self, parafoil, parafoil_state, unsteady_parafoil_state, target, dt):
        ctrl = ControlLogic()
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