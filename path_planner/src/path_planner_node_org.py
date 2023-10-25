#!/usr/bin/env python

import rospy, math, json, time
from sensor_msgs.msg import Joy

## PID code motivated by: Code in lecture slide-4 
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.previous_error = 0
        self.integral_error = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def calc(self, dt, target, current):
        error = target - current
        self.integral_error += error*dt
        derivative_error = (error - self.previous_error)/dt
        u = self.Kp*error + self.Ki*self.integral_error + self.Kd*derivative_error
        self.previous_error = error
        return u

class PathPlanner:
    def  __init__(self, pub, file_name, verbose = False):
        self.pub = pub
        self.file = open(file_name, 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(10)  # 10Hz
        self.dt = 0.1
        self.pid_vx = PID(1, 0.25, 0.1)
        self.pid_vy = PID(1, 0.25, 0.1)
        self.pid_w = PID(1, 0.25, 0.1)
        self.vx_max = 0.3
        self.vx_min = 0.09
        self.vy_max = 0.3
        self.vy_min = 0.12
        self.w_min = 0.7347
        self.w_max = 2.449

    
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

        while (abs(cur_x-target_x) > 0.015) or (abs(cur_y - target_y) > 0.015) or (abs(cur_ori-target_ori)> 0.5):
            ## These are world frame linear and angular velocities
            vx_wf = self.pid_vx.calc(self.dt, target_x, cur_x)
            vy_wf = self.pid_vy.calc(self.dt, target_y, cur_y)
            w_wf = self.pid_w.calc(self.dt, target_ori, cur_ori)

            print('world frame velocities:')
            print(vx_wf, vy_wf, w_wf)

            ## Transforming linear and angular velocities to robo frame
            ## All orientations are in CCW.
            w = w_wf
            vx = vx_wf*math.cos(-1*cur_ori) - vy_wf*math.sin(-1*cur_ori)
            vy = vx_wf*math.sin(-1*cur_ori) + vy_wf*math.cos(-1*cur_ori)

            vx_dir = -1 if vx < 0 else 1
            vy_dir = -1 if vy < 0 else 1
            w_dir = -1 if w < 0 else 1

            ## Handling the fact that the robo has max and min velocities
            vx = min(abs(vx), self.vx_max)
            vy = min(abs(vy), self.vy_max)
            w = min(abs(w), self.w_max)

            if (vx < self.vx_min) and (vx > 0):
                vx = self.vx_min
            if (vy < self.vy_min) and (vy > 0):
                vy = self.vy_min
            if (w < self.w_min) and (w > 0):
                w = self.w_min

            vx = vx*vx_dir
            vy = vy*vy_dir
            w = w*w_dir

            ## Getting back the vx, vy and w for World Frame
            w_wf = w
            vx_wf = vx*math.cos(-1*cur_ori) + vy*math.sin(-1*cur_ori)
            vy_wf = -vx*math.sin(-1*cur_ori) + vy*math.cos(-1*cur_ori) 
            
            msg_count = self.publish(vx, vy, w, msg_count)

            ## giving the bot dt to move actually
            self.rate.sleep()

            ## Updating our position belief in world frame
            cur_x = cur_x + vx_wf*self.dt
            cur_y = cur_y + vy_wf*self.dt
            cur_ori = cur_ori + w_wf*self.dt

            print('final_position:')
            print(cur_x, cur_y, cur_ori)

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
        self.move_to_pose(0, 0, 0, 0.5, 0.5, 1.57)
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
