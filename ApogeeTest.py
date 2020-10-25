import numpy.random as rand
import math
class ApogeeTest:

    def heightRandomizer(self, predApo):
        height = predApo + (rand.uniform(-0.05,0.05) * predApo)
        return height
        
    def calcRadius(self, height):
        radius = height * math.tan(math.radians(15))
        return radius

    def randAzimuth(self):
        return rand.random_sample() * 360

    def calcRandomXYinCircle(self, predApo):
        apogee = [0,0,0,0]
        apogee[2] = self.heightRandomizer(predApo)
        apogee[3] = self.randAzimuth()
        radius = self.calcRadius(apogee[2])
        a = rand.random_sample() * 2 * math.pi
        r = radius * math.sqrt(rand.random_sample())
        apogee[0] = r * math.cos(a)
        apogee[1] = r * math.sin(a)
        return apogee

    def createApogeeArray(self, parafoil_state, points):
        # Create array of random starting points
        apogees = []
        for i in range(0,points):
            apogee = self.calcRandomXYinCircle(parafoil_state['Altitude'])
            apogees.append(apogee)
        return apogees