import numpy as np
np.set_printoptions(suppress=True)

class Switcher:

    def __init__(self):

        # switcher parameters
        self.switchMode = 0 # optimal control

    def setSwitchMode(self, alarm):
        if alarm==1:
            self.switchMode=1

    def getSwithMode(self):
        return self.switchMode