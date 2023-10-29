#!/usr/bin/env python

import rospy, json, time, math
#from std_msgs.msg import String
from sensor_msgs.msg import Joy
from megapi import MegaPi
# import numpy as np

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   
    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
    
        cbl = 0.86 if vbl > 0 else 0.86
        cbr = 0.852 if vbr > 0 else 0.85
        cfl = 0.86 if vfl > 0 else 0.88
        cfr = 0.86 if vfr > 0 else 0.84

        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(cfl*vfl,0))) + 
                  " vfr: " + repr(int(round(cfr*vfr,0))) +
                  " vbl: " + repr(int(round(cbl*vbl,0))) +
                  " vbr: " + repr(int(round(cbr*vbr,0))))

        self.bot.motorRun(self.mfl,cfl*vfl)
        self.bot.motorRun(self.mfr,cfr*vfr)
        self.bot.motorRun(self.mbl,cbl*vbl)
        self.bot.motorRun(self.mbr,cbr*vbr)


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()

class MegaPiControllerWrapper(MegaPiController):
    def __init__(self, port='/dev/ttyUSB0', verbose=False):
        self.verbose = verbose
        MegaPiController.__init__(self, port, False)
    
    def carRotate(self, angle):
        '''
        positive value representing counter clock wise rotation
        '''
        if self.verbose:
            print('Rotate command with angle: ' + str(angle))

        angle = angle*1.0 % 360

        direction = 1 if angle > 0 else -1
        time_to_do = 0.01362* abs(angle) + 0.1817
        MegaPiController.carRotate(self, 45*direction) # HARD CODED rotation speed
        time.sleep(time_to_do)
        MegaPiController.carStop(self)
    
    def carStraight(self, distance):
        '''
        Value of distance in SI units (m)
        '''
        if self.verbose:
            print('Straight command with distance: ' + str(distance))

        direction = 1 if distance > 0 else -1
        if distance > 0:
            time_to_do = 0.3028 + abs(distance)*7.3455
        else:
            time_to_do = 0.3022 + abs(distance)*7.4001
        MegaPiController.carStraight(self, -1*40*direction) # HARD CODED rotation speed
        time.sleep(time_to_do)
        MegaPiController.carStop(self)

class ControllerNode:
    def __init__(self, port='/dev/ttyUSB0', verbose=False, debug=False):
        self.verbose = verbose
        self.debug = debug
        self.controller = MegaPiControllerWrapper(verbose = self.verbose)

    def kinematic_callback(self, vvw):
        vvw = vvw.axes
        #print(vvw[2])
        if int(vvw[2])!=0:
            if self.verbose:
                print(vvw)

            # l = 1.0/(13.5 + 11) #1/(lx + ly)
            # r = 3
            # vvw = np.array([vvw]).reshape([3,1])
            # w = np.linalg.lstsq(np.array([
            #     [1, 1, 1, 1],
            #     [-1, 1, 1, -1],
            #     [-l, l, -l, l],  
            # ])*r/4, vvw)[0]
            # w = w * 100
            # if self.verbose:
            #     print("w:", w)
            # self.setFourMotors(w[0][0], w[1][0], w[2][0], w[3][0])
        
            #cv = -315.0
            #v_straight, v_slide,= cv*vvw[0], cv*vvw[1], 
            #v_rotate = 36.7443*vvw[2]

            ## only one of the angle or distance should 
            ## be non zero at one time
            assert (abs(vvw[0]) > 0.05) ^ (abs(vvw[1]) > 0.05)
              
            if abs(vvw[0]) >= 0.05:
                self.controller.carStraight(vvw[0])
            elif abs(vvw[1]) >=0.05:
                self.controller.carRotate(vvw[1])
            
            if self.verbose:
                print('done!')
            

    def __del__(self):
        #self.controller.carStop()
        pass

if __name__ == "__main__":
    rospy.init_node('controller')
    ctrl_node = ControllerNode(verbose=True)
    time.sleep(1)
    rospy.Subscriber('way_points', Joy, ctrl_node.kinematic_callback, queue_size=3)     
    rospy.spin()
