#!/usr/bin/env python

import rospy, math, json, time
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class PathPlanner:
    def  __init__(self, pub, file_name, verbose = False):
        self.pub = pub
        self.file = open(file_name, 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(5)  # 5Hz
    
    def publish(self, distance, angle, msg_count):
        '''
        Publishes message backbone
        '''
        joy_msg = Joy()
        joy_msg.axes = [distance ,angle  ,msg_count, 0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if self.verbose:
            print(distance, angle)
        self.pub.publish(joy_msg)
        msg_count += 1
        return msg_count

    def publish_rotate(self, angle, msg_count):
        '''
        Publish rotate commands
        '''

        if abs(angle) > 5:
            msg_count = self.publish(0, angle, msg_count)
        return msg_count
    
    def publish_straight(self, dist, msg_count):
        '''
        Publish straight commands
        '''

        if abs(dist) > 0.05:
            msg_count = self.publish(dist, 0, msg_count)
        return msg_count
    
    def straight_time_calculation(self, distance):
        if distance > 0:
            time_to_do = 0.3028 + abs(distance)*7.3455
        else:
            time_to_do = 0.3022 + abs(distance)*7.4001
        return time_to_do

    def rotate_time_calculation(self, angle):
        time_to_do = 0.01362* abs(angle) + 0.1817
        return time_to_do
    
    def run(self): 
        cur_x, cur_y, cur_ori = 0, 0, 0
        msg_count = 0

        ## Sending zeroth message as dummy message
        msg_count = self.publish(0,0,msg_count)
        time.sleep(1)
        
        for line in self.file:
            line = line.split(",")
            ## Scaling the target down by 2
            target_x = float(line[0])/2
            target_y = float(line[1])/2
            target_ori = float(line[2])
            
            if (target_x == cur_x) and (target_y == cur_y) and (target_ori == cur_ori):
                continue
            
            moving_ori = (math.atan2(target_y-cur_y, target_x-cur_x))
            angle_to_rotate = math.degrees(moving_ori-cur_ori)
            dist_factor = 1
            ## rather than rotating 180 and going forward go backwards!
            if angle_to_rotate != 180:
                time_to_sleep = self.rotate_time_calculation(angle_to_rotate)
                msg_count = self.publish_rotate(angle_to_rotate, msg_count)
                time.sleep(time_to_sleep + 2)
            else:
                dist_factor = -1
                moving_ori = cur_ori

            dist_to_travel = math.sqrt((target_y-cur_y)**2 + (target_x-cur_x)**2)
            time_to_sleep = self.straight_time_calculation(dist_factor*dist_to_travel)            
            msg_count = self.publish_straight(dist_factor*dist_to_travel, msg_count)
            time.sleep(time_to_sleep + 0.5)

            angle_to_rotate = math.degrees(target_ori-moving_ori)
            time_to_sleep = self.rotate_time_calculation(angle_to_rotate)
            msg_count = self.publish_rotate(angle_to_rotate, msg_count)
            time.sleep(time_to_sleep + 2)

            cur_x, cur_y, cur_ori = target_x, target_y, target_ori    

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
