#!/usr/bin/env python

from april_detection.msg import AprilTagDetectionArray
import rospy, time
import tf

def april_callback(april_tag_out):

    detections = april_tag_out.detections
    num_detections = len(detections)
    listener = tf.TransformListener()

    for detection in detections:
        #print(type(detection))
        id = detection.id
        print(id)
        body_frame_name = '/camera_' + str(id)

        #try:
        listener.waitForTransform("/world", body_frame_name, rospy.Time(), rospy.Duration(4.0))
        (translation, rotation) = listener.lookupTransform('/world', body_frame_name , rospy.Time(0))
        print(translation)
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    print("Unable to find transform")
    


if __name__ == "__main__":
    rospy.init_node('tf_resolver')
    time.sleep(1)
    rospy.Subscriber('apriltag_detection_array', AprilTagDetectionArray, april_callback, queue_size=3)     
    rospy.spin()