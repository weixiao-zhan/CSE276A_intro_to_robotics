#!/usr/bin/env python  

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "world/marker_0",
                            "world",
        )
        br.sendTransform((0.0, 3.0, 0.0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "world/marker_1",
                            "world",
        )
        rate.sleep()