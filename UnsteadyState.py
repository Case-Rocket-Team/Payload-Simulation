import math
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
        if unsteady_parafoil_state['Azimuth Angle'] > 360:
            unsteady_parafoil_state['Azimuth Angle'] = unsteady_parafoil_state['Azimuth Angle'] - 360
        if unsteady_parafoil_state['Altitude'] < 0:
            print("Downwind (x) Velocity: ", x_velocity, "m/s, Crosswind (y) Velocity: ", y_velocity, "m/s, Vertical Velocity: ", z_velocity, "m/s, Overall Velocity: ", unsteady_parafoil_state['Velocity'], "m/s, Glide Angle", unsteady_parafoil_state['Glide Angle'], "degrees, Azimuth Angle: ", unsteady_parafoil_state['Azimuth Angle'], "degrees, Turning Rate: ", turning_rate, "degrees/s")