#!/usr/bin/env python  

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        br.sendTransform((0.0, 0.8, 0.0),
                            tf.transformations.quaternion_from_euler(0, math.pi/2, -1*math.pi/2, 'rxyz'),
                            rospy.Time.now(),
                            "marker_0",
                            "world",
        )
        '''
        br.sendTransform((0.75, 1.25, 0.0),
                            tf.transformations.quaternion_from_euler(-1*math.pi/2, 0, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_1",
                            "world",
        )

        br.sendTransform((-0.25, 0.0, 0.0),
                            tf.transformations.quaternion_from_euler(math.pi/2, 3*math.pi/2, math.pi, 'rxyz'),
                            rospy.Time.now(),
                            "marker_2",
                            "world",
        
        )

        br.sendTransform((0.0, -0.25, 0.0),
                            tf.transformations.quaternion_from_euler(math.pi/2, 0, math.pi, 'rxyz'),
                            rospy.Time.now(),
                            "marker_3",
                            "world",
        
        )
        '''
        rate.sleep()