import json
import copy
class FileIO:
    def jsonFileRead(self):
        jsonName = input("Enter name of parafoil json file to read from: ")
        with open(jsonName) as f:
            parafoil = json.load(f)
        return parafoil
    
    def massCheck(self):
        return input("Use built in mass for payload and canopy (m) or custom input (c)? ")

    def massInput(self, parafoil):
        parafoil.update({'Payload Mass' : float(input("Enter payload mass (kg): "))})
        parafoil.update({'Canopy Mass' : float(input("Enter parafoil mass (kg): "))})

    def massPick(self, parafoil, parafoil_baked):
        massCheck = self.massCheck()
        if massCheck == "m":
            parafoil.update({'Payload Mass' : parafoil_baked['Payload Mass']})
            parafoil.update({'Canopy Mass' : parafoil_baked['Canopy Mass']})
        elif massCheck == 'c':
            self.massInput(parafoil)
        else:
            print("The hell's wrong with you?")

    def parafoilStateInput(self, parafoil_state):
        parafoil_state['Altitude'] = float(input("Enter starting altitude (m): "))

    def userPrompt(self):
        check = input("Use built in parafoil data (n) or load parafoil data from json (l)? ")
        return check

    def runUI(self, parafoil_baked, parafoil_state):
        check = self.userPrompt()
        if check == "l":
            parafoil = self.jsonFileRead()
            self.massPick(parafoil, parafoil_baked)
            self.parafoilStateInput(parafoil_state)
            return parafoil
        elif check == "n":
            parafoil = copy.deepcopy(parafoil_baked)
            return parafoil
        else:
            print("The hell's wrong with you?")