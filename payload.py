import math

class payload:
    # constants or quantities that are not inputs
    payloadBody = {
        #constants
        'name': "Payload",
        'weight': (4.0894*9.8),
        'CL': 0.84,
        'CD': 0.18,
        'Area': 0.5,
        'dt': 0.1,
        'v0': 20.4,
        #variables
        'V':  11.14,
        'position': [0,0,1], 
        'glideAngle': 0.0,
        'equilibriumGlideAngle': 0.0,
        'azimuthAngle': 0.5, 
        'time': 0.0
    }

    airConditions = {
        #constants
        'MolarMass': 0.0289647,
        'R': 8.31446261815324,
        'T': 21+273.15,
        'po': 1.225,
        #variables
        'p': 0
    }

    #Inputs
    payloadCommands = {
        'BankAngle': 0, 
        'Target': [0,0,0],
        'KP': 10
    }

    def __init__(self):
        print("initialized")
    
    def getAirDensity(self, altitude):
        M = self.airConditions['MolarMass']
        R = self.airConditions['R']
        T = self.airConditions['T']
        po = self.airConditions['po']

        rho = po*math.exp((-9.8*M*altitude)/(R*T))
        return rho

    def getLift(self):
        return 0.5*self.airConditions['p']*math.pow(self.payloadBody['V'], 2)*self.payloadBody['Area']*self.payloadBody['CL']

    def getDrag(self):
        return 0.5*self.airConditions['p']*math.pow(self.payloadBody['V'], 2)*self.payloadBody['Area']*self.payloadBody['CD']

    def calculateVelocity(self):
        airPressureRatio = self.getAirDensity(0)/self.airConditions['p']
        equiVelSQR = airPressureRatio*math.pow(self.payloadBody['v0'],2)
        num = equiVelSQR * math.cos(self.payloadBody['glideAngle'])
        denom = math.cos(self.payloadBody['equilibriumGlideAngle']) * math.cos(self.payloadCommands['BankAngle'])
        self.payloadBody['V'] = math.sqrt((num/denom))
        return self.payloadBody['V']
 
    def getturningRate(self):
        return -9.81*math.tan(self.payloadCommands['BankAngle'])/(math.pow(self.payloadBody['V'], 2)*math.sin(self.payloadBody['glideAngle']))

    def update(self):
        dt = self.payloadBody['dt']
        self.airConditions['p'] = self.getAirDensity(self.payloadBody['position'][2])
        self.payloadBody['equilibriumGlideAngle'] = math.atan(-self.getDrag()/self.getLift())
        self.payloadBody['glideAngle'] = math.atan(math.tan(self.payloadBody['equilibriumGlideAngle']/math.cos(self.payloadCommands['BankAngle'])))
        vel = self.calculateVelocity()
        turnRate = self.getturningRate()
        #heading
        self.payloadBody['azimuthAngle'] +=  dt * turnRate
        # X coordinate
        self.payloadBody['position'][0] += vel*math.cos(turnRate)*math.cos(self.payloadBody['glideAngle'])
        self.payloadBody['position'][1] += vel*math.sin(turnRate)*math.cos(self.payloadBody['glideAngle'])
        self.payloadBody['position'][2] += vel*math.sin(self.payloadBody['glideAngle'])
        self.payloadBody['time'] += dt
