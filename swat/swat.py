import sys,os
sys.path.insert(0,os.getcwd())

from io_plc.IO_PLC import DI_WIFI
from IO import *
from SCADA import H
from plc import plc1,plc2,plc3,plc4,plc5,plc6
from plant.plant import plant

from supervision.deviationDetector import *
from supervision.switcher import *

import numpy as np
np.set_printoptions(suppress=True)

# time is counted in tau seconds, (1.0/tau)*x*y, x unit seconds, y unit minutes
tau = 0.05
maxstep = int(1.0/tau)*60*30
print("tau: {}s, maxstep: {}\n".format(tau, maxstep))

# supervision interval
d = 1.0
intervalNum = int(d/tau)
print("d: {}s, intervalNum: {}\n".format(d, intervalNum))

print("Initializing Plant\n")
init = [563.7481,842.8127,886.7726,381.0458,278.1800] # initIndex=6
print("init: {}\n".format(init))

Plant = plant(tau, init)

print("Defining I/Os\n")
IO_DI_WIFI = DI_WIFI() # Whether PLC processes wireless signal
IO_P1 = P1()           # Define I/Os for sensors and actuators of each process
IO_P2 = P2()
IO_P3 = P3()
IO_P4 = P4()
IO_P5 = P5()
IO_P6 = P6()

print("Initializing SCADA HMI\n")
HMI = H()

print("Initializing PLCs\n")
PLC1 = plc1.plc1(HMI)
PLC2 = plc2.plc2(HMI)
PLC3 = plc3.plc3(HMI)
PLC4 = plc4.plc4(HMI)
PLC5 = plc5.plc5(HMI)
PLC6 = plc6.plc6(HMI)

print("Initializing Supervision\n")
detector = DeviationDetector()
switcher = Switcher()

# control signal and measured water level
U_101 = np.zeros(2)
U_301 = np.zeros(2)
U_401 = np.zeros(2)
list_YS = []

print("Now starting simulation\n")
# Main Loop Body
num = 0
for time in range(0, maxstep):
    # Second, Minute and Hour pulses
    Sec_P = not bool(time%int(1.0/tau))
    Min_P = not bool(time%(int(1.0/tau)*60))

    # Solving out plant odes in 5 ms
    Plant.Actuator(IO_P1,IO_P2,IO_P3,IO_P4,IO_P5,IO_P6,HMI,switcher)
    Plant.Plant(IO_P1,IO_P2,IO_P3,IO_P4,IO_P5,IO_P6,HMI)

    # measured water levels
    if switcher.getSwithMode()==0 and time%intervalNum==0:
        list_YS.append([HMI.LIT101.Pv,HMI.LIT301.Pv,HMI.LIT401.Pv])

    # PLC working
    PLC1.Pre_Main_Raw_Water(IO_P1,HMI)
    PLC2.Pre_Main_UF_Feed_Dosing(IO_P2,HMI)
    PLC3.Pre_Main_UF_Feed(IO_P3,HMI,Sec_P,Min_P)
    PLC4.Pre_Main_RO_Feed_Dosing(IO_P4,HMI)
    PLC5.Pre_Main_High_Pressure_RO(IO_P5,HMI,Sec_P,Min_P)
    PLC6.Pre_Main_Product(IO_P6,HMI)

    # supervision
    if switcher.getSwithMode() == 0:
        if int(time/intervalNum)>1 and time%intervalNum==0:
            print(num)
            # control signal
            U = np.array([[U_101[0], 0.0, 0.0],
                          [U_101[1], 0.0, 0.0],
                          [0.0, U_301[0], 0.0],
                          [0.0, U_301[1], 0.0],
                          [0.0, 0.0, U_401[0]],
                          [0.0, 0.0, U_401[1]]])

            Y = np.array([[list_YS[len(list_YS)-2][0] - list_YS[len(list_YS)-3][0], 0.0, 0.0],
                          [0.0, list_YS[len(list_YS)-2][1] - list_YS[len(list_YS)-3][1], 0.0],
                          [0.0, 0.0, list_YS[len(list_YS)-2][2] - list_YS[len(list_YS)-3][2]]])

            # print(U)
            # print(Y)

            alarm = detector.detector(U, Y)
            # print(detector.getBMean())
            print(alarm)

            for i in range(0, len(alarm)):
                if alarm[i] == 1:
                    switcher.setSwitchMode(alarm[i])
                    break

            num = num + 1

    # control signal
    if switcher.getSwithMode() == 0:
        if time>0 and time%intervalNum == 0:
            U_101 = np.zeros(2)
            U_301 = np.zeros(2)
            U_401 = np.zeros(2)

        # U_101 = [sum(MV101), sum(P101||P102)]
        if IO_P1.MV101.DO_Open == 1:
            U_101[0] = round(U_101[0] + tau, 3)
        if IO_P1.P101.DO_Start == 1 or IO_P1.P102.DO_Start == 1:
            U_101[1] = round(U_101[1] + tau, 3)

        # U_301 = [sum(MV201&&(P101||P102)), sum(P301||P302)]
        if IO_P2.MV201.DO_Open == 1 and (IO_P1.P101.DO_Start == 1 or IO_P1.P102.DO_Start == 1):
            U_301[0] = round(U_301[0] + tau, 3)
        if IO_P3.P301.DO_Start == 1 or IO_P3.P302.DO_Start == 1:
            U_301[1] = round(U_301[1] + tau, 3)

        # U_401 = [sum((P301||P302)&&~MV301&&MV302&&~MV303&&~MV304&&~P602),
        #          sum(P401||P402)]
        if (IO_P3.P301.DO_Start == 1 or IO_P3.P302.DO_Start == 1) and \
            IO_P3.MV301.DO_Open == 0 and IO_P3.MV302.DO_Open == 1 and IO_P3.MV303.DO_Open == 0 and IO_P3.MV304.DO_Open == 0 and \
            IO_P6.P602.DO_Start == 0:
            U_401[0] = round(U_401[0] + tau, 3)
        if IO_P4.P401.DO_Start == 1 or IO_P4.P402.DO_Start == 1:
            U_401[1] = round(U_401[1] + tau, 3)