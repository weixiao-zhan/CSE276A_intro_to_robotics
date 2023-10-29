#!/usr/bin/env python

import rospy, math, json, time
from sensor_msgs.msg import Joy
import numpy as np

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = dt
        self.maximumValue = 0.1

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

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

        return result
    

class PathPlanner:
    def  __init__(self, pub, file_name, verbose = False):
        self.pub = pub
        self.file = open(file_name, 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(10)  # 10Hz
        self.dt = 0.1
        self.pid = PIDcontroller(0.02, 0.005, 0.005, 0.1)

    def handle_frame_transforms(self, twist, current_state):
        J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
        return np.dot(J, twist)

    
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

    def move_to_pose(self, cur_x, cur_y, cur_ori, target_x, target_y, target_ori):
        
        ## Publishing first dummy message 
        msg_count = 0 
        msg_count = self.publish(0,0,0,msg_count)

        ## Setting the target for PID
        self.pid.setTarget(target_x, target_y, target_ori)
        current_state = np.array([cur_x, cur_y, cur_ori], dtype='f')

        while (abs(current_state[0]-target_x) > 0.05) or (abs(current_state[1] - target_y) > 0.05) or (abs(current_state[2]-target_ori)> 0.5):
            update_value = self.pid.update(current_state)
            update_values_robo_frame = self.handle_frame_transforms(update_value, current_state)
            msg_count = self.publish(update_values_robo_frame[0], update_values_robo_frame[1], update_values_robo_frame[2], msg_count)
            self.rate.sleep()
            current_state += update_value*self.dt

        ## Stopping the bot
        msg_count = self.publish(0,0,0,msg_count)
    

    def run(self):
        '''
        ## Initial Position
        cur_x, cur_y, cur_ori = 0, 0, 0
        for line in self.file:
            line = line.split(",")
            target_x = float(line[0])
            target_y = float(line[1])
            target_ori = float(line[2]) ## This is Positive in CCW
            
            if (target_x == cur_x) and (target_y == cur_y) and (target_ori == cur_ori):
                continue

            self.move_to_pose(cur_x, cur_y, cur_ori, target_x, target_y, target_ori)

            cur_x = target_x
            cur_y = target_y
            cur_ori = target_ori
        '''
            
        ## Stopping after all waypoints have been traversed
        self.move_to_pose(0.5, 0, 0, 0.5, 1, 3.14)
        self.stop()
    
    def stop(self):
        self.file.close()
        print("path plan all published")

if __name__ == "__main__":
    pub = rospy.Publisher("way_points", Joy, queue_size=6)
    rospy.init_node("path_planner")    
    node = PathPlanner(pub, "/root/rb5_ws/src/rb5_ros/path_planner/src/waypoints.txt", True)
    node.run()
    rospy.spin()
