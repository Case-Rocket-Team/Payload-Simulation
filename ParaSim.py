import math
import copy
import json
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from SteadyState import SteadyState
from UnsteadyState import UnsteadyState
from FileIO import FileIO
from ApogeeTest import ApogeeTest
from Graphing import Graphing
g = 9.81
density = 1.225
# Defined Constants for our parafoil
parafoil_baked = {
    'Span' : 1.016,                     # m
    'Chord' : 0.508,                    # m
    'Canopy Mass' : 0.049,              # kg
    'Payload Mass' : 4, #3,                 # kg
    'Vehicle Coefficients' : {
        'CL' : 0.8,
        'CD' : 0.2,
    }
}
# State of the vehicle
parafoil_state = {
    'X-position' : 0,                   # m
    'Y-position' : 0,                   # m 
    'Altitude' : 0,                     # m
    'Glide Angle' : -23.3,              # degrees
    'Bank Angle' : 0,                   # degrees
    'Azimuth Angle' : 0,                # degrees from x-axis
    'Velocity' : 10.95,                 # m/s
    'Wind Field': [0, 0, 0]
}

# (x,y)
target = (0,0)

def main():
    # Call classes
    apo = ApogeeTest()
    # steady = SteadyState()
    unsteady = UnsteadyState()
    fio = FileIO()
    # Prompt user to determine whether to use built in data or change to file and user input
    parafoil, control_prompt, ctrl_check = fio.runUI(parafoil_baked, parafoil_state)

    # Create a copy of the vehicle state for the unsteady state equations
    base_parafoil_state = copy.deepcopy(parafoil_state)

    dt = 0.1
    if control_prompt == 'y':
        # Path follow test
        if ctrl_check == 'p':
            graph = Graphing(1)
            kp, ki , kd = 0.75 , 0.001, 0 
            unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
            unsteady_parafoil_state['X-position'] = -2000
            unsteady_parafoil_state['Altitude'] = 2000
            unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_mags, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections, proportionals, integrals, derivatives, waypoints = unsteady.waypointFollower(parafoil, parafoil_state, unsteady_parafoil_state, target, kp, ki, kd, dt)
            graph.path_graphing(target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, waypoints, proportionals, integrals, derivatives)

        # Return to Pad approach with random start points
        if ctrl_check == 'r':
            # Proportional Gain constants, integral gain, derivative gain
            # 1 for circling controller, 2 for approach controller
            '''
            kp1, kp2 = 1, 1
            ki1, ki2 = 0.001, 0
            kd1, kd2 = 32, 0
            '''
            kp1, kp2 = 1, 1
            ki1, ki2 = 0.0001, 0
            kd1, kd2 = 32, 0

            # apogees = apo.createApogeeArray(parafoil_state, 10)
            apogees = [[0,0,1000,0]]
            graph = Graphing(len(apogees))

            unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_mags, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections, count_terminator, count = [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees),  [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees), [0] * len(apogees)
            proportionals, integrals, derivatives = [0] * len(apogees), [0] * len(apogees), [0] * len(apogees)
            

            for i in range(0, len(apogees)):
                print("Iteration ", i)
                unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
                unsteady_parafoil_state['X-position'] = apogees[i][0]
                unsteady_parafoil_state['Y-position'] = apogees[i][1]
                unsteady_parafoil_state['Altitude'] = apogees[i][2]
                unsteady_parafoil_state['Azimuth Angle'] = apogees[i][3]
                unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], unsteady_angles[i], unsteady_times[i], deltas[i], unsteady_mags[i], unsteady_azimuths[i], unsteady_bank_angles[i], left_servo_angles[i], right_servo_angles[i], deflections[i], count_terminator[i], count[i], proportionals[i], integrals[i], derivatives[i] = unsteady.runRTPControlLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, kp1, kp2,  ki1, ki2, kd1, kd2, dt)

            hits = 0
            spirals = 0
            for i in range(0, len(apogees)):
                listSize = len(unsteady_x_positions[i])-1
                if math.sqrt(math.pow(unsteady_x_positions[i][listSize] - target[0], 2) + math.pow(unsteady_y_positions[i][listSize] - target[1], 2)) < 10:
                    hits += 1
                if math.sqrt(math.pow(unsteady_x_positions[i][listSize] - target[0], 2) + math.pow(unsteady_y_positions[i][listSize] - target[1], 2)) > 250:
                    spirals += 1
                    flag = "!!!ALERT!!!"
                else:
                    flag = ""
                print(flag, " Apogee: ", apogees[i], " Distance From Target:" + str(math.sqrt(math.pow(unsteady_x_positions[i][listSize] - target[0], 2) + math.pow(unsteady_y_positions[i][listSize] - target[1], 2))))
            print(hits, " hits out of ", len(apogees))
            print(spirals, " spirals out of ", len(apogees))

            graph.varying_apogee_graphing(apogees, parafoil_state, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_mags, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections, count_terminator, count, proportionals, integrals, derivatives)

        # Straight approach with varying kp
        if ctrl_check == 's':
            # Proportional Gain constant
            kp = [100, 50, 25, 10, 1, 0.1]
            ki, kd = 0, 0

            graph = Graphing(len(kp))

            unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections = [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp),  [0] * len(kp), [0] * len(kp),  [0] * len(kp), [0] * len(kp), [0] * len(kp)
            unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
            
            for i in range(0, len(kp)):
                unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
                print(i)
                unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], unsteady_angles[i], unsteady_times[i], deltas[i], unsteady_azimuths[i], unsteady_bank_angles[i], left_servo_angles[i], right_servo_angles[i], deflections[i] = unsteady.runStraightControlLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, kp[i], kd, ki, dt)
            
            for i in range(0, len(kp)):
                listSize = len(unsteady_x_positions[i])-1
                print("KP: " + str(kp[i]) + " Distance From Target:" + str(math.sqrt(math.pow(unsteady_x_positions[i][listSize] - target[0], 2) + math.pow(unsteady_y_positions[i][listSize] - target[1], 2))))

            graph.varying_gain_graphing(kp, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections)

        # Testing circularization w/ target dist control
        if ctrl_check == 'd':
            # Proportional Gain constant
            kp = [2]
            ki, kd = 0, 20
            graph = Graphing(len(kp))

            unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, unsteady_azimuths, unsteady_bank_angles, unsteady_mags = [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp), [0] * len(kp),  [0] * len(kp), [0] * len(kp)
            unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
            
            for i in range(0, len(kp)):
                unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
                print(i)
                unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], unsteady_angles[i], unsteady_times[i], unsteady_azimuths[i], unsteady_bank_angles[i], unsteady_mags[i] = unsteady.runDistToTargControlLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, kp[i], ki, kd, dt)
            
            # for i in range(0, len(kp)):
            #     listSize = len(unsteady_x_positions[i])-1
            #     print("KP: " + str(kp[i]) + " Distance From Target:" + str(math.sqrt(math.pow(unsteady_x_positions[i][listSize] - target[0], 2) + math.pow(unsteady_y_positions[i][listSize] - target[1], 2))))

            graph.varying_gain_graphing(kp, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, unsteady_azimuths, unsteady_bank_angles, unsteady_mags)
            

    elif control_prompt == 'n':
        graph = Graphing(1)
        unsteady_parafoil_state = copy.deepcopy(base_parafoil_state)
        unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths = unsteady.runLoop(parafoil, parafoil_state, unsteady_parafoil_state, target, dt)
        graph.uncontrolled_graphing(target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths)


if __name__ == "__main__":
    main() 