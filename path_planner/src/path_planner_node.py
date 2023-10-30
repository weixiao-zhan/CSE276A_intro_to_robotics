#!/usr/bin/env python

import rospy, math, time, threading
from datetime import datetime
from sensor_msgs.msg import Joy
import pickle
import numpy as np

## PID code motivated by: Code in lecture slide-4 
class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = dt
        self.maximumValue = 0.4 # ToDo
        self.minimumValue = 0.3

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result
    
    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0
        elif(resultNorm < self.minimumValue):
            result = (result / resultNorm) * self.minimumValue
            # self.I = 0.0
        return result

def handle_frame_transforms(vvw, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                [0.0,0.0,1.0]])
    return np.dot(J, vvw)

class PathPlanner:
    def  __init__(self, verbose = False):
        self.pub = rospy.Publisher("/bot_vvw", Joy, queue_size=6)
        self.file = open("/root/rb5_ws/src/rb5_ros/path_planner/src/waypoints.txt", 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(20)  # 10Hz
        self.dt = 0.05
        self.pid = PID(0.5, 0.025, 0.025, self.dt)

        self.lock = threading.Lock()
        self.cur_x = 0
        self.cur_y = 0
        self.cur_ori = 0
        self.x_locs = []
        self.y_locs = []
        self.most_recent_update = datetime.now()
    
    def bot_loc_callback(self, bot_loc):
        bot_loc = bot_loc.axes
        if bot_loc[0] == 1:
            if self.verbose:
                print("get loc from camera loop", bot_loc[1], bot_loc[2], bot_loc[3])
            with self.lock:
                self.cur_x = bot_loc[1]
                self.cur_y = bot_loc[2]
                self.cur_ori = bot_loc[3]
                self.most_recent_update = datetime.now()

    def publish(self, vx, vy, w, msg_count):
        '''
        Publishes velocities
        '''
        joy_msg = Joy()
        joy_msg.axes = [vx ,vy ,w ,msg_count ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if self.verbose:
            print('published velocities:')
            print(vx, vy, w)
        self.pub.publish(joy_msg)
        msg_count += 1
        return msg_count

    def move_to_pose(self, target_x, target_y, target_ori):
        
        ## Zero error for all PIDs
        self.pid.setTarget(target_x, target_y, target_ori)

        ## Publishing first dummy message 
        msg_count = 0 
        msg_count = self.publish(0,0,0,msg_count)

        while np.linalg.norm(self.pid.getError(np.array([self.cur_x, self.cur_y, self.cur_ori]), np.array([target_x, target_y, target_ori]))) > 0.05:
            with self.lock:
                cur_x, cur_y, cur_ori = self.cur_x, self.cur_y, self.cur_ori
            
            ## These are world frame linear and angular velocities
            vvw_wf = self.pid.update(np.array([cur_x, cur_y, cur_ori]))
            print('world frame velocities:', vvw_wf)

            vvw = handle_frame_transforms(vvw_wf, [cur_x, cur_y, cur_ori])
            
            msg_count = self.publish(vvw[0], vvw[1], vvw[2], msg_count)

            ## giving the bot dt to move actually
            self.rate.sleep()

            ## Updating our position belief in world frame
            with self.lock:
                if (datetime.now() - self.most_recent_update).total_seconds() > 0.5*self.dt:
                    self.cur_x = cur_x + vvw_wf[0]*self.dt
                    self.cur_y = cur_y + vvw_wf[1]*self.dt
                    self.cur_ori = cur_ori + vvw_wf[2]*self.dt

            print('final_position:')
            
            # logging
            with self.lock:
                self.x_locs.append(self.cur_x)
                self.y_locs.append(self.cur_y)
                print(self.cur_x, self.cur_y, self.cur_ori)

        ## Stopping the bot
        msg_count = self.publish(0,0,0,msg_count)
    

    def run(self):
        # Initial Position
        for line in self.file:
            line = line.split(",")
            target_x = float(line[0])
            target_y = float(line[1])
            target_ori = float(line[2]) ## This is Positive in CCW
            
            if (target_x == self.cur_x) and (target_y == self.cur_y) and (target_ori == self.cur_ori):
                continue

            self.move_to_pose(target_x, target_y, target_ori)

        ## Stopping after all waypoints have been traversed
        #self.move_to_pose(0.5, 1, 3.141)
        self.stop()
        exit()
    
    def stop(self):
        self.file.close()
        print("path plan all published")

if __name__ == "__main__":
    rospy.init_node("path_planner")  

    pub = rospy.Publisher("/bot_vvw", Joy, queue_size=6)
    pp = PathPlanner(True)
    
    rospy.Subscriber('/bot_loc', Joy, pp.bot_loc_callback, queue_size=3)
    time.sleep(1)
    pp.run()
    
    with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" 'x_locs'  + ".pkl", 'rb') as f:
        pickle.dump(pp.x_locs, f)
    with open("/root/rb5_ws/src/rb5_ros/path_planner/src/" 'y_locs'  + ".pkl", 'rb') as f:
        pickle.dump(pp.y_locs, f)

    rospy.spin()
