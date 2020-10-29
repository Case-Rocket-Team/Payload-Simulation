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
        parafoil_state['Altitude'] = float(input("Enter starting altitude (meters): "))

    def userPrompt(self):
        check = input("Load parafoil data from json file (l)? ")
        return check
    def controlPrompt(self):
        check = input("Use P controller (y/n)? ")
        ctrl_check = input("Use Return to Pad (r), distance to pad control (d), or Straight Approach (s)?")
        return check, ctrl_check

    def runUI(self, parafoil_baked, parafoil_state):
        check = self.userPrompt()
        if check == "l":
            parafoil = self.jsonFileRead()
            self.massPick(parafoil, parafoil_baked)
            self.parafoilStateInput(parafoil_state)
            p_check, c_check = self.controlPrompt()
            return parafoil, p_check, c_check
        elif check == "n":
            parafoil = copy.deepcopy(parafoil_baked)
            p_check, c_check = None, None
            return parafoil, p_check, c_check
        else:
            print("The hell's wrong with you?")