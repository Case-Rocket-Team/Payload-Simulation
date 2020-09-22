import math
import copy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from SteadyState import SteadyState
from UnsteadyState import UnsteadyState
g = 9.81
density = 1.225
# Defined Constants for our parafoil
parafoil = {
    'Canopy Area' : 0.72,           # m^2
    'Canopy Mass' : 0.5,            # kg
    'Payload Mass' : 4,             # kg
    'Coffecient of Drag' : 0.1,
    'Coeffecient of Lift' : 0.8,
}
# State of the vehicle
parafoil_state = {
    'X-position' : 0,               # m
    'Y-position' : 0,               # m 
    'Altitude' : 1400,              # m
    'Glide Angle' : -7.12,          # degrees
    'Bank Angle' : 0,               # degrees
    'Azimuth Angle' : 45,            # degrees from x-axis
    'Velocity' : 11.14,             # m/s
}
def main():
    steady = SteadyState()
    unsteady = UnsteadyState()
    # Create a copy of the vehicle state for the unsteady state equations
    unsteady_parafoil_state = copy.deepcopy(parafoil_state)
    dt = 0.1
    time = 0
    times = [0]
    angles = [0]
    x_positions = [parafoil_state["X-position"]]
    y_positions = [parafoil_state["Y-position"]]
    altitudes = [parafoil_state['Altitude']]
    # Run loop that iterates the kinematic steady state equations
    while parafoil_state['Altitude'] > 0:
        #print("Time is: ", time)
        #print("Coords (X,Y,Z): (", parafoil_state['X-position'], ", ", parafoil_state['Y-position'], ", ", parafoil_state['Altitude'], ")")
        steady.updatePosition(parafoil, parafoil_state, dt)
        time += dt
        times.append(time)
        angles.append(parafoil_state['Glide Angle'])
        x_positions.append(parafoil_state['X-position'])
        y_positions.append(parafoil_state['Y-position'])
        altitudes.append(parafoil_state['Altitude'])
    print("^^^Time is: ", time)
    unsteady_times = [0]
    unsteady_angles = [0]
    unsteady_x_positions = [unsteady_parafoil_state["X-position"]]
    unsteady_y_positions = [unsteady_parafoil_state["Y-position"]]
    unsteady_altitudes = [unsteady_parafoil_state['Altitude']]
    unsteady_time = 0
    # Run loop taht iterates the kinematic unsteady state equations
    while unsteady_parafoil_state['Altitude'] > 0:
        #print("Time is: ", unsteady_time)
        unsteady.updatePosition(parafoil, unsteady_parafoil_state, dt)
        if unsteady_time > 250 and unsteady_time < 251:
            unsteady_parafoil_state['Bank Angle'] = 5
        elif unsteady_time > 750 and unsteady_time < 751:
            unsteady_parafoil_state['Bank Angle'] = 0
        #print("Coords (X,Y,Z): (",unsteady_parafoil_state['X-position'], ", ", unsteady_parafoil_state['Y-position'], ", ", unsteady_parafoil_state['Altitude'], ")")
        unsteady_time += dt
        unsteady_times.append(unsteady_time)
        unsteady_angles.append(unsteady_parafoil_state['Glide Angle'])
        unsteady_x_positions.append(unsteady_parafoil_state['X-position'])
        unsteady_y_positions.append(unsteady_parafoil_state['Y-position'])
        unsteady_altitudes.append(unsteady_parafoil_state['Altitude'])
    print("^^^Time is: ", unsteady_time)
    plt.plot(x_positions, altitudes, label = "Steady State")
    plt.plot(unsteady_x_positions, unsteady_altitudes, label = "Unsteady State")
    plt.xlabel("Downrange Distance (m)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude and Downrange Distance")
    plt.legend()

    plt.figure()
    plt.plot(times, altitudes, label = "Steady State")
    plt.plot(unsteady_times, unsteady_altitudes, label = "Unsteady State")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude vs Time")
    plt.legend()

    plt.figure()
    plt.plot(times, angles, label = "Steady State")
    plt.plot(unsteady_times, unsteady_angles, label = "Unsteady State")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle of Flight")
    plt.title("Glide Angle vs Time")
    plt.legend()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax = plt.axes(projection='3d')
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    ax.plot(unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, c = 'Green', label = "Unsteady State")
    #ax.plot(x_positions, y_positions, altitudes, c = 'Blue', label = "Steady State")
    ax.set_xlabel("Downwind X (m)")
    ax.set_ylabel("Crosswind Y (m)")
    ax.set_zlabel("Altitude Z (m)")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main() 