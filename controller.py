from payload import payload
import math

class controller:
    def __init__(self):
        print("controller initialized")

    def getBankAngle(self, load):
        delta = self.getAngleDelta(load.payloadCommands['Target'], load.payloadBody['azimuthAngle'], load.payloadBody['position'])
        deltaBankAngle = delta * load.payloadCommands['BankAngle']
        load.payloadCommands['BankAngle'] = deltaBankAngle
        return deltaBankAngle

    def getAngleDelta(self, target, azimuthAngle, position):
        dx = (target[0] - position[0])
        dy = (target[1] - position[1])
        mag = math.sqrt(dx * dx + dy * dy)
        #print("dx, dy, mag: ", dx, " ", dy, " ", math.acos(mag/dx))
        delta = math.acos((dx * math.cos(azimuthAngle) + dy * math.sin(azimuthAngle) )/ mag)
        return delta