from payload import payload
from controller import controller
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def main():
    coordsT = []
    coordsX = []
    coordsZ = []
    coordsY = []
    coordsRho = []
    coordsBank = []
    coordsV = []

    load = payload()
    cont = controller()
    load.payloadBody['position'][2] = 1400
    load.payloadBody['azimuthAngle'] = 0
    load.payloadCommands['BankAngle'] = 0

    while(load.payloadBody['position'][2] > 0):
        #load.payloadCommands['BankAngle'] += 0.05
        coordsT.append(load.payloadBody['time'])
        coordsX.append(load.payloadBody['position'][0])
        coordsY.append(load.payloadBody['position'][1])
        coordsZ.append(load.payloadBody['position'][2])
        coordsV.append(load.payloadBody['V'])
        coordsBank.append(load.payloadCommands['BankAngle'])
        print("Time", load.payloadBody['time'], "( ",load.payloadBody['position'][0], " ", load.payloadBody['position'][1], " ", load.payloadBody['position'][2], ")" , "Bank Angle: ", load.payloadCommands['BankAngle'], "Vel: ", load.payloadBody['V'])
        load.update()
        cont.getBankAngle(load)
        coordsRho.append(load.airConditions['p'])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(coordsX, coordsY,coordsZ)
    fig2 = plt.figure()
    ax2 = fig2.add_subplot()
    ax2.plot(coordsZ, coordsRho)
    fig3 = plt.figure()
    ax3 = fig3.add_subplot()
    ax3.plot(coordsX, coordsY)
    fig4 = plt.figure()
    ax4 = fig4.add_subplot()
    ax4.plot(coordsBank, coordsV)


    ax.set_xlabel(r'Down Range - X (m)', fontsize = 15)
    ax.set_ylabel(r'Cross-Range - Y (m)', fontsize = 15)
    ax.set_zlabel(r'Altitude - Z (m)', fontsize = 15)
    ax.set_title('Parafoil Trajectory')
    ax.grid(True)

    ax2.set_xlabel(r'Altitude - Z (m)', fontsize = 15)
    ax2.set_ylabel(r'Air Density(kg/m^3)', fontsize = 15)
    ax2.set_title('Air Density Estimation')
    ax2.grid(True)

    ax3.set_xlabel(r'Down Range - X (m)', fontsize = 15)
    ax3.set_ylabel(r'Cross-Range - Y (m)', fontsize = 15)
    ax3.set_title('Horizontal Trajectory')
    ax3.grid(True)

    ax4.set_ylabel(r'Velocity (m/s)', fontsize = 15)
    ax4.set_xlabel(r'Bank Angle (rads)', fontsize = 15)
    ax4.set_title('Bank Angle VS Velocity')
    ax4.grid(True)


    plt.show()



if __name__ == "__main__":
    main()