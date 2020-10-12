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

initialVelocity = 20.4

# time in seconds
dt = 0.1

# coef of lift/drag
CL = 0.8
CD = 0.2
CT = math.sqrt(CD*CD + CL*CL)
# 4kg(payload) + 0.5kg (parafoil)
W = 9.8*(4 + 0.5)
# density kg/m^3
po = 1.225
# Area m^2
A = 0.72

#meters
radius = 30

vT = 0
# angle heading
heading = 0.0
    
p = 1.225 #kg/m^2
R = 8.31446261815324  #  m3⋅Pa⋅K−1⋅mol−1(edited)
T = 21+273.15 #K
M = 0.0289647 #g/mol(edited)
g = 9.8 #m/s^2
8.31446261815324   # kg⋅m2·K−1⋅mol−1s−2

parafoil = {
    'Canopy Area' : 0.5,            # m^2
    'Canopy Mass' : 0.0894,         # kg
    'Payload Mass' : 4,             # kg
    'Coffecient of Drag' : 0.18,
    'Coeffecient of Lift' : 0.84,
}
# State of the vehicle
parafoil_state = {
    'X-position' : 0,               # m
    'Y-position' : 0,               # m 
    'Altitude' : 1400,              # m
    'Glide Angle' : -12,          # degrees
    'Bank Angle' : 0,               # degrees
    'Azimuth Angle' : 0,            # degrees from x-axis
    'Velocity' : 11.14,             # m/s
}

def getAirDensity(altitude):
    rho = po*math.exp((-g*M*altitude)/(R*T))
    return rho

def printCoords(t, x, y, z, glide):
    print("Time",t,": (",x, ",", y ,',', z,")","glide ", math.degrees(glide))

def calculateLift(vel, p):
    return 0.5*p*vel*vel*A*CL

def calculateDrag(vel, p):
    return 0.5*p*vel*vel*A*CD

def calculateEquilibriumGlideAngle(vel, p):
    return math.atan(-calculateDrag(vel, p) / calculateLift(vel, p))

def calculateGlideAngle(bankAngle, vel, p):
    #return 1/math.atan(-calculateDrag(vel, p)/(calculateLift(vel, p)*math.cos(bankAngle)))
    return math.atan(math.tan(calculateEquilibriumGlideAngle(vel, p))/math.cos(bankAngle))

def calculateEquilibriumVelocity(alt, v0, h0):
    airPressureRatio = getAirDensity(h0)/getAirDensity(alt)
    return math.sqrt(airPressureRatio*v0*v0)

def calculateVelocity(equiVel, glideAngle, equiGlideAngle, bankAngle):
    num = equiVel*equiVel * math.cos(glideAngle)
    denom = math.cos(equiGlideAngle) * math.cos(bankAngle)
    print('num:', num,'denom: ', denom, 'glideAngle:' , glideAngle, 'equil glide Angle: ', equiGlideAngle)
    if(num/denom > 0):
        return math.sqrt((num/denom))
    else:
        return 0

def calculateTurningVel(velocity, bankAngle, glideAngle):
    return -9.81*math.tan(bankAngle)/(velocity*velocity*math.sin(glideAngle))

def calculateXVel(velocity, turnRate, glideAngle):
    return velocity*math.cos(turnRate)*math.cos(glideAngle)

def calculateYVel(velocity, turnRate, glideAngle):
    return velocity * math.sin(turnRate)*math.cos(glideAngle)

def calculateAltitudeVel(vel, glideAngle):
    return vel*math.sin(glideAngle)

def quasiSteadyStateEstimation():
    px = 0
    py = 0
    pz = 1400
    pxy = 0
    time = 0
    bankAngle = 0
    turningRate = 0
    azimuthAngle = 0.0
    lastVelocity = initialVelocity
    coordsT = []
    coordsX = []
    coordsZ = []
    coordsY = []
    coordsRho = []
    while (pz > 0 and pz < 4000):

        p = getAirDensity(pz)
        L = calculateLift(lastVelocity, p)
        D = calculateDrag(lastVelocity, p)
        EquilibtriumGlideAngle = calculateEquilibriumGlideAngle(pz, p)
        GlideAngle = calculateGlideAngle(bankAngle, lastVelocity, p)
        EquilibriumVelocity = calculateEquilibriumVelocity(pz, initialVelocity, 0)
        velocity = calculateVelocity(EquilibriumVelocity, GlideAngle, EquilibtriumGlideAngle, bankAngle)
        turningRate = calculateTurningVel(velocity, bankAngle, GlideAngle)
        #print(turningRate)

        azimuthAngle += turningRate*dt
        px += calculateXVel(velocity, azimuthAngle, GlideAngle)*dt
        py += calculateYVel(velocity, azimuthAngle, GlideAngle)*dt
        pz += calculateAltitudeVel(velocity, GlideAngle)* dt
        time += dt
        coordsT.append(time)
        coordsX.append(px)
        coordsY.append(py)
        coordsZ.append(pz)
        coordsRho.append(p)
        printCoords(time, px, py, pz, turningRate)


        lastVelocity = velocity

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(coordsX, coordsY,coordsZ)
    fig2 = plt.figure()
    ax2 = fig2.add_subplot()
    ax2.plot(coordsZ, coordsRho)

    ax2.set_xlabel(r'Altitude - Z (m)', fontsize = 15)
    ax2.set_ylabel(r'Air Density(kg/m^3)', fontsize = 15)
    ax2.set_title('Air Density Estimation')
    ax2.grid(True)


    ax.set_xlabel(r'Down Range - X (m)', fontsize = 15)
    ax.set_ylabel(r'Cross-Range - Y (m)', fontsize = 15)
    ax.set_zlabel(r'Altitude - Z (m)', fontsize = 15)
    ax.set_title('Parafoil Trajectory')
    ax.grid(True)

    plt.show()
        


def steadyStateEstimation():
    p = 1.225
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
        glidePathAngle = calculateEquilibriumGlideAngle(v, p)
        vxy = v * math.cos(glidePathAngle)
        #print('turning speed: ', vxy/radius)
        vz = v*math.sin(glidePathAngle)

        if(time > 10 and time < 30):
            heading = 3.1415*time/radius
        else:
            heading = 0
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
        #coordsRho.append(p)
    
    #graph printouts

    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(coordsX, coordsY,coordsZ)

    ax.set_xlabel(r'Down Range (m)', fontsize = 15)
    ax.set_ylabel(r'Cross-Range (m)', fontsize = 15) 
    ax.set_zlabel(r'Altitude (m)', fontsize = 15)
    ax.set_title('3D position graph')
    ax.grid(True)

    plt.show()


def main():
    quasiSteadyStateEstimation()


if __name__ == "__main__":
    main()
