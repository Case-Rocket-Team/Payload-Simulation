import math
import copy
import json
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from SteadyState import SteadyState
from UnsteadyState import UnsteadyState
from FileIO import FileIO
g = 9.81
density = 1.225
# Defined Constants for our parafoil
parafoil_baked = {
    'Span' : 1.016,                     # m
    'Chord' : 0.508,                    # m
    'Canopy Mass' : 0.049,              # kg
    'Payload Mass' : 3,                 # kg
    'Vehicle Coefficients' : {
        'CL' : 0.8,
        'CD' : 0.2,
    }
}
# State of the vehicle
parafoil_state = {
    'X-position' : 0,               # m
    'Y-position' : 0,               # m 
    'Altitude' : 1400,              # m
    'Glide Angle' : -14,            # degrees
    'Bank Angle' : -1,               # degrees
    'Azimuth Angle' : 0,            # degrees from x-axis
    'Velocity' : 10.95,             # m/s
}
target = (3000,3000)      # (x,y)
def main():
    steady = SteadyState()
    unsteady = UnsteadyState()
    fio = FileIO()
    # Prompt user to determine whether to use built in data or change to file and user input
    parafoil, control_prompt = fio.runUI(parafoil_baked, parafoil_state)
    # Create a copy of the vehicle state for the unsteady state equations
    base_parafoil_state = copy.deepcopy(parafoil_state)
    dt = 0.1
    x_positions, y_positions, altitudes, angles, times = steady.runLoop(parafoil, parafoil_state, dt)
    if control_prompt == 'y':
        # Proportional Gain constant
        # kp = [1]
        # col = ["b","g","r","c","m","y"]
        # unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles = [0], [0], [0], [0], [0], [0], [0], [0]
        # for i in range(0, len(kp)):
        #     unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
        #     print(i)
        #     unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], unsteady_angles[i], unsteady_times[i], deltas[i], unsteady_azimuths[i], unsteady_bank_angles[i] = unsteady.runControlLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, kp[i], dt)
        unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
        kp = 0.25
        unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles = unsteady.runControlLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, kp, dt)
        

        # plt.plot(x_positions, altitudes, label = "Steady State")
        # plt.plot(unsteady_x_positions, unsteady_altitudes, label = "Unsteady State")
        # plt.xlabel("Downrange Distance (m)")
        # plt.ylabel("Altitude (m)")
        # plt.title("Altitude and Downrange Distance")
        # plt.legend()

        # plt.figure()
        # plt.plot(times, altitudes, label = "Steady State")
        # plt.plot(unsteady_times, unsteady_altitudes, label = "Unsteady State")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Altitude (m)")
        # plt.title("Altitude vs Time")
        # plt.legend()

        plt.figure()
        # for i in range(0, len(kp) - 1):
        #     plt.plot(unsteady_x_positions[i], unsteady_y_positions[i], label = "Unsteady State, kp:" + str(kp[i]), c = col[i])
        plt.plot(unsteady_x_positions, unsteady_y_positions, label = "Unsteady State, kp:" + str(kp))
        plt.scatter(target[0], target [1], c = "r", s = 10)
        plt.xlabel("X-position (m)")
        plt.ylabel("Y-position (m)")
        plt.title("X-position vs Y-position")
        plt.axis('square')
        plt.legend()

        # plt.figure()
        # for i in range(0, len(kp) - 1):
        #     plt.plot(unsteady_times[i], unsteady_azimuths[i], label = "Unsteady State, kp:" + str(kp[i]), c = col[i])
        # plt.xlabel("Time (s)")
        # plt.ylabel("Azimuth Angle")
        # plt.title("Azimuth vs Time")
        # plt.legend()

        plt.figure()
        # for i in range(0, len(kp) - 1):
        #     plt.plot(unsteady_times[i], unsteady_bank_angles[i], label = "Unsteady State, kp:" + str(kp[i]), c = col[i])
        plt.plot(unsteady_times, unsteady_bank_angles, label = "Unsteady State, kp:" + str(kp))
        plt.xlabel("Time (s)")
        plt.ylabel("Bank Angle (deg)")
        plt.title("Bank Angle vs Time")
        plt.legend()

        plt.figure()
        # for i in range(0, len(kp) - 1):
        #     plt.plot(unsteady_times[i], deltas[i], label = "Unsteady State, kp:" + str(kp[i]), c = col[i])
        plt.plot(unsteady_times, deltas, label = "Unsteady State, kp:" + str(kp))
        plt.plot([0,unsteady_times[-1]], [0,0], label = "Setpoint", c = "r")
        plt.xlabel("Time (s)")
        plt.ylabel("Delta from Azimuth")
        plt.title("Delta vs Time")
        plt.legend()

        # plt.figure()
        # plt.plot(times, angles, label = "Steady State")
        # plt.plot(unsteady_times, unsteady_angles, label = "Unsteady State")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Angle of Flight")
        # plt.title("Glide Angle vs Time")
        # plt.legend()

        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
        #ax.plot(x_positions, y_positions, altitudes, c = 'Blue', label = "Steady State")
        # for i in range(0, len(kp) - 1):
        #     ax.plot(unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], c = col[i], label = "Unsteady State, kp:" + str(kp[i]))
        ax.plot(unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, label = "Unsteady State, kp:" + str(kp))
        ax.scatter(target[0], target[1], 0, c = 'Red', label = "Target", s = 10)
        ax.set_title("Parafoil Path with a P Controller")
        ax.set_xlabel("Downwind X (m)")
        ax.set_ylabel("Crosswind Y (m)")
        ax.set_zlabel("Altitude Z (m)")
        ax.legend()
        plt.show()
    elif control_prompt == 'n':
        unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
        unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths = unsteady.runLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, dt)
        # plt.plot(x_positions, altitudes, label = "Steady State")
        # plt.plot(unsteady_x_positions, unsteady_altitudes, label = "Unsteady State")
        # plt.xlabel("Downrange Distance (m)")
        # plt.ylabel("Altitude (m)")
        # plt.title("Altitude and Downrange Distance")
        # plt.legend()

        # plt.figure()
        # plt.plot(times, altitudes, label = "Steady State")
        # plt.plot(unsteady_times, unsteady_altitudes, label = "Unsteady State")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Altitude (m)")
        # plt.title("Altitude vs Time")
        # plt.legend()

        plt.figure()
        plt.plot(unsteady_x_positions, unsteady_y_positions, label = "Unsteady State", c = "b")
        plt.scatter(target[0], target [1], c = "r", label = "Target", s = 10)
        plt.xlabel("X-position (m)")
        plt.ylabel("Y-position (m)")
        plt.title("X-position vs Y-position")
        plt.legend()

        plt.figure()
        plt.plot(unsteady_times, unsteady_azimuths, label = "Unsteady State", c = "b")
        plt.xlabel("Time (s)")
        plt.ylabel("Azimuth Angle (deg)")
        plt.title("Azimuth vs Time")
        plt.legend()

        plt.figure()
        plt.plot(unsteady_times, deltas, label = "Unsteady State", c = "b")
        plt.xlabel("Time (s)")
        plt.ylabel("Delta from Azimuth (deg)")
        plt.title("Delta vs Time")
        plt.legend()

        # plt.figure()
        # plt.plot(times, angles, label = "Steady State")
        # plt.plot(unsteady_times, unsteady_angles, label = "Unsteady State")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Angle of Flight")
        # plt.title("Glide Angle vs Time")
        # plt.legend()

        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
        #ax.plot(x_positions, y_positions, altitudes, c = 'Blue', label = "Steady State")
        ax.plot(unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, c = "b", label = "Unsteady State")
        ax.scatter(target[0], target[1], 0, c = 'Red', label = "Target", s = 10)
        ax.set_title("Parafoil Path")
        ax.set_xlabel("Downwind X (m)")
        ax.set_ylabel("Crosswind Y (m)")
        ax.set_zlabel("Altitude Z (m)")
        ax.legend()
        plt.show()
    


if __name__ == "__main__":
    main() 