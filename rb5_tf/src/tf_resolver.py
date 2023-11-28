#!/usr/bin/env python

from april_detection.msg import AprilTagDetectionArray
import rospy, time
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
import math

def april_callback(april_tag_out):

    detections = april_tag_out.detections
    listener = tf.TransformListener()

    num_detections = 0.0
    x_avg = 0.0
    y_avg = 0.0
    theta_avg = 0.0
    weight_sum = 0.0
    

    for detection in detections:
        id = detection.id
        body_frame_name = '/body_' + str(id)

        if id > 7:
            continue

        try:
            listener.waitForTransform("/world", body_frame_name, rospy.Time(), rospy.Duration(1))
            (translation, rotation) = listener.lookupTransform("/world", body_frame_name , rospy.Time(0))
            rotation = euler_from_quaternion(rotation, 'sxyz')

            weight_curr_marker = 1/(translation[0]**2 + translation[1]**2)

            x_avg += translation[0]*weight_curr_marker
            y_avg += translation[1]*weight_curr_marker
            theta_avg += rotation[2]*weight_curr_marker
            weight_sum += weight_curr_marker
            num_detections += 1
            
        except Exception:
            print("Unable to find transform")

    joy_msg = Joy()
    joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
    
    if num_detections !=0:
        x_avg /= weight_sum
        y_avg /= weight_sum
        theta_avg /= weight_sum
        joy_msg.axes = [1.0, x_avg ,y_avg ,theta_avg ,0.0 ,0.0 ,0.0 ,0.0]
        pub.publish(joy_msg)
    else:
        joy_msg.axes = [-1.0, 0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]

    print(joy_msg.axes)


if __name__ == "__main__":
    rospy.init_node('tf_resolver')
    pub = rospy.Publisher("/bot_loc", Joy, queue_size=6)
    time.sleep(1)
    rospy.Subscriber('apriltag_detection_array', AprilTagDetectionArray, april_callback, queue_size=3)     
    rospy.spin()