#!/usr/bin/env python

import rospy, math, time, threading
from datetime import datetime
from sensor_msgs.msg import Joy

## PID code motivated by: Code in lecture slide-4 
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.previous_error = 0
        self.integral_error = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def zero_error(self):
        self.integral_error = 0
        self.previous_error = 0

    def calc(self, dt, target, current):
        error = target - current
        self.integral_error += error*dt
        derivative_error = (error - self.previous_error)/dt
        u = self.Kp*error + self.Ki*self.integral_error + self.Kd*derivative_error
        self.previous_error = error
        return u

class PathPlanner:
    def  __init__(self, verbose = False):
        self.pub = rospy.Publisher("/bot_vvw", Joy, queue_size=6)
        self.file = open("/root/rb5_ws/src/rb5_ros/path_planner/src/waypoints.txt", 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(20)  # 10Hz
        self.dt = 0.05
        self.pid_vx = PID(0.5, 0.05, 0.05)
        self.pid_vy = PID(0.5, 0.05, 0.05)
        self.pid_w = PID(0.06, 0.006, 0.006)

        self.vx_max = 0.15
        self.vx_min = 0.09
        self.vy_max = 0.15
        self.vy_min = 0.12
        self.w_min = 0.7347
        self.w_max = 2.449

        self.lock = threading.Lock()
        self.cur_x = 0
        self.cur_y = 0
        self.cur_ori = 0
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
        self.pid_vx.zero_error()
        self.pid_vy.zero_error()
        self.pid_w.zero_error()

        ## Publishing first dummy message 
        msg_count = 0 
        msg_count = self.publish(0,0,0,msg_count)


        while (abs(self.cur_x-target_x) > 0.05) or (abs(self.cur_y - target_y) > 0.05) or (abs(self.cur_ori-target_ori)> 0.1):
            with self.lock:
                cur_x, cur_y, cur_ori = self.cur_x, self.cur_y, self.cur_ori
            
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

            vx, vy, w = abs(vx), abs(vy), abs(w)

            if vx > self.vx_max:
                vx = self.vx_max
            elif vx < self.vx_min/2:
                vx = 0
            elif vx < self.vx_min:
                vx = self.vx_min
            
            if vy > self.vy_max:
                vy = self.vy_max
            elif vy < self.vy_min/2:
                vy = 0
            elif vy < self.vy_min:
                vy = self.vy_min

            if w > self.w_max:
                w = self.w_max
            elif w < self.w_min/2:
                w = 0
            elif w < self.w_min:
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
            with self.lock:
                if (datetime.now() - self.most_recent_update).total_seconds() > 0.5*self.dt:

                    self.cur_x = cur_x + vx_wf*self.dt
                    self.cur_y = cur_y + vy_wf*self.dt
                    self.cur_ori = cur_ori + w_wf*self.dt

            print('final_position:')
            print(self.cur_x, self.cur_y, self.cur_ori)


        ## Stopping the bot
        msg_count = self.publish(0,0,0,msg_count)
    

    def run(self):
        ## Initial Position
        '''
        for line in self.file:
            line = line.split(",")
            target_x = float(line[0])
            target_y = float(line[1])
            target_ori = float(line[2]) ## This is Positive in CCW
            
            if (target_x == self.cur_x) and (target_y == self.cur_y) and (target_ori == self.cur_ori):
                continue

            self.move_to_pose(target_x, target_y, target_ori)
        '''
        
        ## Stopping after all waypoints have been traversed
        self.move_to_pose(0.5, 0, 0)
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
    rospy.spin()
