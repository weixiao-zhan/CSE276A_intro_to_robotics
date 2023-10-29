#!/usr/bin/env python

import rospy, time, math
#from std_msgs.msg import String
from sensor_msgs.msg import Joy
from megapi import MegaPi
import numpy as np

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class ControllerNode:
    def __init__(self, port='/dev/ttyUSB0', verbose=False, debug=False):
        self.verbose = verbose
        self.debug = debug

        self.bot = MegaPi()
        self.bot.start(port = port)
        time.sleep(1)
        self.port = port
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   
        self.kinematic_model = np.array([[1, -1, -0.1225], [1, 1, 0.1225], [1, 1, -0.1225], [1, -1, 0.1225]])/0.03

        if verbose:
            self.printConfiguration()
    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0.0, vfr=0.0, vbl=0.0, vbr=0.0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))

        cbl = 0.86 if vbl > 0 else 0.86
        cbr = 0.852 if vbr > 0 else 0.85
        cfl = 0.86 if vfl > 0 else 0.88
        cfr = 0.86 if vfr > 0 else 0.84
        
        if self.verbose:
            print("Set Motors (actual): vfl: " + repr((round(cfl*vfl,1))) + 
                  " vfr: " + repr(int(round(cfr*vfr,1))) +
                  " vbl: " + repr(int(round(cbl*vbl,1))) +
                  " vbr: " + repr(int(round(cbr*vbr,1))))

        self.bot.motorRun(self.mfl,cfl*vfl)
        self.bot.motorRun(self.mfr,cfr*vfr)
        self.bot.motorRun(self.mbl,cbl*vbl)
        self.bot.motorRun(self.mbr,cbr*vbr)

    def kinematic_callback(self, vvw):
        vvw = vvw.axes
        if vvw[3]!=0:
            if self.verbose:
                print(vvw)

            vx, vy, w = vvw[0], vvw[1], vvw[2]
            vel_arr = np.array([vx, vy, w]).T
            omegas = np.matmul(self.kinematic_model, vel_arr)

            ## the constant to convert actual omegas to motor commands
            omegas *= 10
            omegas = omegas.tolist()
            print(omegas)

            ## Moving the motors
            ## Our motors are winded such that all of them move in CCW 
            ## given a positive motor command  
            self.setFourMotors(
                omegas[3],
                -1*omegas[2],
                omegas[1],
                -1*omegas[0]
            )
            
        
    def __del__(self):
        self.bot.close()

if __name__ == "__main__":
    rospy.init_node('controller')
    ctrl_node = ControllerNode(verbose=True)
    time.sleep(1)
    rospy.Subscriber('/bot_vvw', Joy, ctrl_node.kinematic_callback, queue_size=3)     
    rospy.spin()
