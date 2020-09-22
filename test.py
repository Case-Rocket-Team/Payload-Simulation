import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# TODO:
# add non steady state EQs 
# Add turning w/ non steady state EQs

#variables
#position in meters
glidePathAngle = -0.0

# v in m/s
vx = 0.1
vy = 0.1
vz = 0.1
#v = 20.4

#constants

maxVelocity = 30 # ft/s

# time in seconds
dt = 0.1

# coef of lift/drag
CL = 0.8
CD = 0.1
CT = math.sqrt(CD*CD + CL*CL)
# 4kg(payload) + 0.5kg (parafoil)
W = 9.8*(4 + 0.5)
# density kg/m^3
p = 1.225 
# Area m^2
A = 0.72

#meters
radius = 30

vT = 0
# angle heading
heading = 0.0
    
po = 1.225 #kg/m^2
R = 8.31446261815324  #  m3⋅Pa⋅K−1⋅mol−1(edited)
T = 21+273.15 #K
M = 0.0289647 #g/mol(edited)
g = 9.8 #m/s^2
8.31446261815324   # kg⋅m2·K−1⋅mol−1s−2
def getAirDensity(altitude):
    rho = po*math.exp((-g*M*altitude)/(R*T))
    return rho

def printCoords(t, x, y, z, glide):
    print("Time",t,": (",x, ",", y ,',', z,")", "Vz ", vz,"glide ", math.degrees(glide))

def calculateLift(vel):
    return 0.5*p*vel*vel*A*CL

def calculateDrag(vel):
    return 0.5*p*vel*vel*A*CD

def calculateEquilibriumGlideAngle(vel):
    return 1/math.atan(-calculateDrag(vel) / calculateLift(vel))

def calculateGlideAngle(bankAngle, vel):
    return 1/math.atan(-calculateDrag(vel)/(calculateLift(vel)*math.cos(bankAngle)))

def calculateEquilibriumVelocity(alt, v0, h0):
    airPressureRatio = getAirDensity(h0)/getAirDensity(alt)
    return math.sqrt(airPressureRatio*v0*v0)

def calculateVelocity(equiVel, glideAngle, equiGlideAngle, bankAngle):
    num = equiVel*equiVel * math.cos(bankAngle)
    denom = math.cos(equiGlideAngle) * math.cos(bankAngle)
    return math.sqrt(num/denom)

def calculateTurningAccel(velocity, bankAngle, glideAngle):
    return -9.81*math.tan(bankAngle)/(velocity*velocity*math.sin(glideAngle))

def calculateXVel(turnRate, glideAngle):
    return -math.cos(turnRate)/math.tan(glideAngle)

def calculateYVel(turnRate, glideAngle):
    return -math.sin(turnRate)/math.tan(glideAngle)

def calculateAltitude(vel, glideAngle):
    return vel*math.sin*(glideAngle)


def main():
    px = 0
    py = 0
    pz = 1400
    pxy = 0
    time = 0
    coordsT = []
    coordsX = []
    coordsZ = []
    coordsY = []
    coordsRho = []
    while (pz > 0 or pz > 2000):
        #p = getAirDensity(pz)
        v = math.sqrt(2*W/(p*A*CT))
        glidePathAngle = calculateGlideAngle(v)
        vxy = v * math.cos(glidePathAngle)
        print('turning speed: ', vxy/radius)
        vz = v*math.sin(glidePathAngle)

        heading = 3.1415*time/radius
        vx = math.cos(heading) * vxy
        vy = math.sin(heading) * vxy
        px += dt * vx
        py += dt * vy
        pxy += dt * vxy
        pz += dt * vz
        time += dt
        #printCoords(time, px, py, pz, glidePathAngle)
        coordsT.append(time)
        coordsX.append(px)
        coordsY.append(py)
        coordsZ.append(pz)
        coordsRho.append(p)
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(coordsX, coordsY,coordsZ)
    ax2.scatter(coordsZ, coordsRho)

    ax.set_xlabel(r'Down Range (m)', fontsize = 15)
    ax.set_ylabel(r'Cross-Range (m)', fontsize = 15)
    ax.set_zlabel(r'Altitude (m)', fontsize = 15)
    ax.set_title('3D position graph')
    ax.grid(True)

    ax2.set_xlabel(r'Altitude(meters)', fontsize = 15)
    ax2.set_ylabel(r'Air denity(Kg/m^3)', fontsize = 15)
    ax2.set_title('Air density vs altitude')
    ax2.grid(True)

    plt.show()
    
if __name__ == "__main__":
    main()
