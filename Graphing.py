from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

class Graphing:
    def __init__(self, length):
        self.col = []
        for i in range(0, length, 6):
            self.col.append("b")
            self.col.append("g")
            self.col.append("r")
            self.col.append("c")
            self.col.append("m")
            self.col.append("y")


    # Got this bad boy off the web
    def set_axes_equal(self, ax):
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        '''
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    # Graphing call for loop with varying apogees/start points
    def varying_apogee_graphing(self, apogees, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles, step_time, step, left_servo_angles, right_servo_angles, deflections):
        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_x_positions[i], unsteady_y_positions[i], label = "Apogee:" + str(i), c = self.col[i])
        plt.scatter(target[0], target [1], c = "r", s = 10)
        plt.xlabel("X-position (m)")
        plt.ylabel("Y-position (m)")
        plt.title("X-position vs Y-position")
        plt.axis('square')
        plt.legend()

        fig, ax = plt.subplots()
        circle = plt.Circle((target[0], target[1]), 10, fill = False, color = 'k')
        for i in range(0, len(apogees)):
            plt.scatter(unsteady_x_positions[i][-1], unsteady_y_positions[i][-1], c = 'b')
        plt.scatter(target[0], target[1], label = "Target", c = 'r')
        ax.add_artist(circle)
        plt.xlabel("X axis (m)")
        plt.ylabel("Y axis (m)")
        plt.title("Ground Hit Points")
        plt.axis('square')
        plt.legend()

        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_times[i], deflections[i], label = "Unsteady State", c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Flap Deflection (deg)")
        plt.title("Flap Deflection vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_times[i], left_servo_angles[i], label = "Left Servo", c = 'blue')
            plt.plot(unsteady_times[i], right_servo_angles[i], label = "Right Servo", c = 'red')
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title("Servo Angle vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_times[i], unsteady_azimuths[i], label = "Apogee:" + str(i), c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Azimuth Angle")
        plt.title("Azimuth vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_times[i], unsteady_bank_angles[i], label = "Apogees:" + str(i), c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Bank Angle (deg)")
        plt.title("Bank Angle vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(apogees)):
            plt.plot(unsteady_times[i][0:-20], deltas[i][0:-20], label = "Apogees: " + str(i), c = self.col[i])
        for i in range(0, len(apogees)):
            plt.plot([0, step_time[i], step_time[i], 300], [step[i], step[i], 0, 0], label = "Setpoint Step " + str(i), c = 'y')
        plt.xlabel("Time (s)")
        plt.ylabel("Delta from Azimuth")
        plt.title("Delta vs Time")
        plt.legend()

        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
        for i in range(0, len(apogees)):
            ax.plot(unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], c = self.col[i], label = "Apogees: " + str(i))
        ax.scatter(target[0], target[1], 0, c = 'Red', label = "Target", s = 10)
        ax.set_title("Parafoil Path with a P Controller")
        ax.set_xlabel("Downwind X (m)")
        ax.set_ylabel("Crosswind Y (m)")
        ax.set_zlabel("Altitude Z (m)")
        self.set_axes_equal(ax)
        ax.legend()
        plt.show()

    # Graphing call for running control with varying gains
    def varying_gain_graphing(self, kp, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths, unsteady_bank_angles, left_servo_angles, right_servo_angles, deflections):
        plt.figure()
        for i in range(0, len(kp)):
            plt.plot(unsteady_x_positions[i], unsteady_y_positions[i], label = "Unsteady State, kp:" + str(kp[i]), c = self.col[i])
        plt.scatter(target[0], target [1], c = "r", s = 10)
        plt.xlabel("X-position (m)")
        plt.ylabel("Y-position (m)")
        plt.title("X-position vs Y-position")
        plt.axis('square')
        plt.legend()

        plt.figure()
        for i in range(0, len(kp)):
            plt.plot(unsteady_times[i], unsteady_azimuths[i], label = "Unsteady State, kp:" + str(kp[i]), c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Azimuth Angle")
        plt.title("Azimuth vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(kp)):
            plt.plot(unsteady_times[i], deflections[i], label = "Unsteady State, kp:" + str(kp[i]), c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Flap Deflection (deg)")
        plt.title("Flap Deflection vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(kp)):
            plt.plot(unsteady_times[i], left_servo_angles[i], label = "Left Servo kp:" + str(kp[i]), c = 'blue')
            plt.plot(unsteady_times[i], right_servo_angles[i], label = "Right Servo kp:" + str(kp[i]), c = 'red')
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.title("Servo Angle vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(kp)):
                plt.plot(unsteady_times[i], unsteady_bank_angles[i], label = "Unsteady State, kp:" + str(kp[i]), c = self.col[i])
        plt.xlabel("Time (s)")
        plt.ylabel("Bank Angle (deg)")
        plt.title("Bank Angle vs Time")
        plt.legend()

        plt.figure()
        for i in range(0, len(kp)):
            plt.plot(unsteady_times[i], deltas[i], label = "Unsteady State, kp:" + str(kp[i]), c = self.col[i])
        plt.plot([0,300], [0,0], label = "Setpoint", c = "y")
        plt.xlabel("Time (s)")
        plt.ylabel("Delta from Azimuth")
        plt.title("Delta vs Time")
        plt.legend()

        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
        for i in range(0, len(kp)):
            ax.plot(unsteady_x_positions[i], unsteady_y_positions[i], unsteady_altitudes[i], c = self.col[i], label = "Unsteady State, kp:" + str(kp[i]))
        ax.scatter(target[0], target[1], 0, c = 'Red', label = "Target", s = 10)
        ax.set_title("Parafoil Path with a P Controller")
        ax.set_xlabel("Downwind X (m)")
        ax.set_ylabel("Crosswind Y (m)")
        ax.set_zlabel("Altitude Z (m)")
        self.set_axes_equal(ax)
        ax.legend()
        plt.show()

    # Graphing call for running loop without control
    def uncontrolled_graphing(self, target, unsteady_x_positions, unsteady_y_positions, unsteady_altitudes, unsteady_angles, unsteady_times, deltas, unsteady_azimuths):
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
        self.set_axes_equal(ax)
        ax.legend()
        plt.show()    