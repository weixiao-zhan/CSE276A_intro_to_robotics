#!/usr/bin/env python

import rospy
import math
import time
import threading
from datetime import datetime
from sensor_msgs.msg import Joy
import numpy as np
import tf

class KF:
    def __init__(self, n):
        self.Xk = np.zeros((n,1))  # Start from Origin
        self.Sigmak = np.identity(n)*0.001
        self.Fk = np.identity(n)
        self.Qk = np.identity(n)

        self.Qk[0][0] = 0.02 
        self.Qk[1][1] = 0.02
        self.Qk[2][2] = 0.001

        self.Rk = np.identity(n)
        self.Rk *= 0.0001

        self.Hk = None
        self.lm_ordering = []

    def compose_Zk(self, zk):
        """
        Compose [Xlm, Ylm]'s into Zk
        Also, updating Hk
        """

        Zk = []
        for id in self.lm_ordering:
            if id in zk:
                Zk.extend(zk[id])

        return np.array(Zk).reshape(-1, 1)
    
    def getHk(self, Xknew, zk, Zk):
        """
        Usedf to compute Hk, given Theta_Robot and Zk
        """
        
        theta_r = Xknew[2,0]
        Hk = np.zeros((Zk.shape[0], Xknew.shape[0]))
        cosine_theta = np.cos(theta_r)
        sine_theta = np.sin(theta_r)

        # Used to get the row to update
        counter = 0

        for i in range(len(self.lm_ordering)):
            tag_id = self.lm_ordering[i]
            if tag_id in zk:
                row1 = 2*counter 
                row2 = 2*counter + 1

                column1 = 2*i + 3
                column2 = 2*i + 4

                # Updating entries corresponding to xr, yr, thetar
                Hk[row1, 0] = -1*cosine_theta
                Hk[row1, 1] = -1*sine_theta
                Hk[row2, 0] = sine_theta
                Hk[row2, 1] = cosine_theta

                # Updating entries corresponding to xlm, ylm
                Hk[row1, column1] = cosine_theta
                Hk[row1, column2] = sine_theta
                Hk[row2, column1] = -1*sine_theta
                Hk[row2, column2] = cosine_theta

                counter += 1

        self.Hk = Hk
        return

    def predict(self, dx, dy, dtheta):
        """
        Prediction step of Kalman Filter
        """

        # Performing an predict on bot locations.
        # Landmark Locations are stationary so, not updated.
        Xknew = self.Xk.copy()
        Xknew[0][0] = Xknew[0][0] + dx
        Xknew[1][0] = Xknew[1][0] + dy
        Xknew[2][0] = Xknew[2][0] + dtheta

        # Updating the uncertainities of the State vector.
        Sigmaknew = np.matmul(np.matmul(self.Fk, self.Sigmak), self.Fk.T) + self.Qk
        
        return Xknew, Sigmaknew
    
    def update(self, Zk, Xknew, Sigmaknew, zk):
        """
        Update step of Kalman Filter
        """

        if Zk.shape[0] == 0:
            self.Xk = Xknew
            self.Sigmak = Sigmaknew
            return

        self.getHk(Xknew, zk, Zk)

        # Computing Kalman Gain and other matrices.
        Yk = Zk - np.matmul(self.Hk, Xknew)
        Sk = np.matmul(np.matmul(self.Fk, Sigmaknew), self.Fk.T) + self.Rk
        Kk = np.matmul(np.matmul(Sigmaknew, self.Hk.T), np.linalg.inv(Sk))

        # Updating the State and Uncertanities based on KF.
        Xknewfinal = Xknew + np.matmul(Kk, Yk)
        Sigmaknewfinal = np.matmul((np.identity(Sigmaknew.shape[0]) - np.matmul(Kk, self.Hk)), Sigmaknew)

        self.Xk = Xknewfinal
        self.Sigmak = Sigmaknewfinal

    def update_matrices(self, zk):  
        """
        Updating the matrices Xk, Sigmak, Fk, Qk, Rk
        """

        for id in zk.keys():
            if id not in self.lm_ordering:
                self.lm_ordering.append(id)

        new_dim_size = 3 + len(self.lm_ordering)
        self.Rk = np.identity(new_dim_size)*0.0001
        self.Fk = np.identity(new_dim_size)
        self.Qk = np.identity(new_dim_size)*0.03

        self.Qk[0][0] = 0.02 
        self.Qk[1][1] = 0.02
        self.Qk[2][2] = 0.001

        #TODO: Updating Xk, Sigmak

        return
        
class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.timestep = dt
        self.maximumValue = 0.35  # ToDo
        self.minimumValue = 0.20

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
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
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
        if (resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0
        elif (resultNorm < self.minimumValue):
            result = (result / resultNorm) * self.minimumValue
            # self.I = 0.0
        return result


def handle_frame_transforms(vvw, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0, 0.0, 1.0]])
    return np.dot(J, vvw)

class tf_resolver:
    def __init(self):
        self.listener = tf.TransformListener()
        self.markers = set()
        self.new_markers = set()
    
    def getZk(self):
        '''
        look into /tf, return Zk array and index array
        '''
        self.new_markers.clear()
        Zk = {}
        for idx in range(0,8):
            marker_name = "marker_" + str(idx)
            if self.listener.frameExists(marker_name):
                if idx not in self.markers:
                    self.markers.add(idx)
                    self.new_markers.add(idx)
                self.listener.waitForTransform("/body", marker_name, rospy.Time(), rospy.Duration(1))
                (translation, rotation) = self.listener.lookupTransform("/body", marker_name, rospy.Time(0))
                Zk[idx] = translation[0:2] # marker_i.x in body frame
        return Zk
    
class PathPlanner:
    def __init__(self, verbose=False):
        self.pub = rospy.Publisher("/bot_vvw", Joy, queue_size=6)
        self.file = open(
            "/root/rb5_ws/src/rb5_ros/path_planner/src/waypoints.txt", 'r')
        self.verbose = verbose
        self.rate = rospy.Rate(20)  # 10Hz
        self.dt = 0.05
        self.pid = PID(0.40, 0.010, 0.030, self.dt)
        self.tf_resolver = tf_resolver()
        self.kf = KF(3)

        self.logging_x = []
        self.logging_y = []

    def publish(self, vx, vy, w, msg_count):
        '''
        Publishes velocities
        '''
        joy_msg = Joy()
        joy_msg.axes = [vx, vy, w, msg_count, 0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        if self.verbose:
            print('published velocities:')
            print(vx, vy, w)
        self.pub.publish(joy_msg)
        msg_count += 1
        return msg_count

    def move_to_pose(self, target_x, target_y, target_ori):
        # rest PID
        self.pid.setTarget(target_x, target_y, target_ori)

        # Publishing first dummy message
        msg_count = 0
        msg_count = self.publish(0, 0, 0, msg_count)

        while np.linalg.norm(self.pid.getError(self.kf.Xk[0:3, 0], np.array([target_x, target_y, target_ori]))) > 0.04:
            cur_x, cur_y, cur_ori = self.kf.Xk[0, 0], self.kf.Xk[1, 0], self.kf.Xk[2, 0]

            # These are world frame linear and angular velocities
            vvw_wf = self.pid.update(np.array([cur_x, cur_y, cur_ori]))
            if self.verbose:
                print('world frame velocities:', vvw_wf)
            vvw = handle_frame_transforms(vvw_wf, [cur_x, cur_y, cur_ori])
            msg_count = self.publish(vvw[0], 1.25*vvw[1], vvw[2], msg_count)

            # giving the bot dt to move actually
            self.rate.sleep()

            dx = vvw_wf[0]*self.dt
            dy = vvw_wf[1]*self.dt
            dtheta = vvw_wf[2]*self.dt

            zk = self.tf_resolver.getZk()  ## Dict
            Zk = self.kf.compose_Zk(zk)

            Xknew, Sigmaknew = self.kf.predict(dx, dy, dtheta)
            self.kf.update(Zk, Xknew, Sigmaknew, zk)
            self.kf.update_matrices(zk)

        
        # Stopping the bot
        msg_count = self.publish(0, 0, 0, msg_count)

    def run(self):
        # Initial Position
        for line in self.file:
            line = line.split(",")
            target_x = float(line[0])
            target_y = float(line[1])
            target_ori = float(line[2])  # This is Positive in CCW

            self.move_to_pose(target_x, target_y, target_ori)
            time.sleep(5)

        # Stopping after all waypoints have been traversed
        self.stop()
        exit()

    def stop(self):
        # TODO: Add logging file saving 
        self.file.close()
        print("path plan all published")


if __name__ == "__main__":
    rospy.init_node("path_planner")
    pub = rospy.Publisher("/bot_vvw", Joy, queue_size=6)
    pp = PathPlanner(verbose=True)
    time.sleep(1)
    pp.run()
    rospy.spin()
