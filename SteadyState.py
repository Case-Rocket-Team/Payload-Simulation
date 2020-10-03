import math

g = 9.81
density = 1.225
class SteadyState:
    # Calculate total velocity magnitude
    def calcVelocity(self, parafoil, parafoil_state, lift_force, drag_force, dt):
        weight = (parafoil['Payload Mass'] + parafoil['Canopy Mass']) * g
        ct = math.sqrt((parafoil['Vehicle Coefficients']['CD'] ** 2) + (parafoil['Vehicle Coefficients']['CL'] ** 2))
        velocity = math.sqrt((2 / density) * (weight / (parafoil['Span'] * parafoil['Chord']) * (1 / ct)))
        parafoil_state['Velocity'] = velocity
    # Calculate Lift Force
    def calcLiftForce(self, parafoil, parafoil_state):
        lift_force = 0.5 * density * (parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CL']
        return lift_force
    # Calculate Drag Force
    def calcDragForce(self, parafoil, parafoil_state):
        drag_force = 0.5 * density * (parafoil_state['Velocity'] ** 2) * (parafoil['Span'] * parafoil['Chord']) * parafoil['Vehicle Coefficients']['CD']
        return drag_force
    # Calculate Glide Angle
    def calcGlideAngle(self, parafoil, parafoil_state, lift_force, drag_force, dt):
        glide_angle = math.degrees(math.atan(- drag_force / lift_force))
        parafoil_state['Glide Angle'] = glide_angle
    # Calculate Turning Velocity
    # def calcTurningVelocity():
    #     return 30
    # Run loop
    def updatePosition(self, parafoil, parafoil_state, dt):
        lift_force = self.calcLiftForce(parafoil, parafoil_state)
        drag_force = self.calcDragForce(parafoil, parafoil_state)
        self.calcGlideAngle(parafoil, parafoil_state, lift_force, drag_force, dt)
        self.calcVelocity(parafoil, parafoil_state, lift_force, drag_force, dt)
        x_velocity = parafoil_state['Velocity'] * math.cos(math.radians(parafoil_state['Glide Angle'])) * math.cos(math.radians(parafoil_state['Azimuth Angle']))
        parafoil_state['X-position'] += x_velocity * dt
        y_velocity = parafoil_state['Velocity'] * math.cos(math.radians(parafoil_state['Glide Angle'])) * math.sin(math.radians(parafoil_state['Azimuth Angle']))
        parafoil_state['Y-position'] += y_velocity * dt
        h_velocity = parafoil_state['Velocity'] * math.sin(math.radians(parafoil_state['Glide Angle']))
        parafoil_state['Altitude'] += h_velocity * dt
        #parafoil_state['Azimuth Angle'] += self.calcTurningVelocity() * dt
        if parafoil_state['Altitude'] < 0:
            print("Downwind (x) Velocity: ", x_velocity, "Crosswind (y) Velocity: ", y_velocity, "Vertical Velocity: ", h_velocity, "Velocity: ", parafoil_state['Velocity'], "Glide Angle: ", parafoil_state['Glide Angle'], "Glide Ratio: ", 1400 / parafoil_state['X-position'])
