#!/usr/bin/env python  

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('static_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    print("static_tf_broadcaster is running")
    while not rospy.is_shutdown():
        br.sendTransform((3, 1, 0.0),
                            tf.transformations.quaternion_from_euler(0, math.pi/2, -1*math.pi/2, 'rxyz'),
                            rospy.Time.now(),
                            "marker_3",
                            "world",
        )
        br.sendTransform((3, 2, 0.0),
                            tf.transformations.quaternion_from_euler(0, math.pi/2, -1*math.pi/2, 'rxyz'),
                            rospy.Time.now(),
                            "marker_2",
                            "world",
        )

        br.sendTransform((2, 3, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_4",
                            "world",
        )
        br.sendTransform((1, 3, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_5",
                            "world",
        )
        
        br.sendTransform((0.0, 1, 0.0),
                            tf.transformations.quaternion_from_euler(-1*math.pi/2, -1*math.pi/2, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_0",
                            "world",
        )
        br.sendTransform((0.0, 2, 0.0),
                            tf.transformations.quaternion_from_euler(-1*math.pi/2, -1*math.pi/2, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_1",
                            "world",
        )

        br.sendTransform((1, 0.0, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2,math.pi,0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_6",
                            "world",
        )
        br.sendTransform((2, 0.0, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2,math.pi,0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_7",
                            "world",
        )


        br.sendTransform((1.95, 1.5, 0.0),
                            tf.transformations.quaternion_from_euler(-1*math.pi/2, -1*math.pi/2, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_8",
                            "world",
        )
        br.sendTransform((1.5, 1.675, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2,math.pi,0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_9",
                            "world",
        )
        br.sendTransform((1.275, 1.50, 0.0),
                            tf.transformations.quaternion_from_euler(0, math.pi/2, -1*math.pi/2, 'rxyz'),
                            rospy.Time.now(),
                            "marker_10",
                            "world",
        )
        br.sendTransform((1.50, 1.325, 0.0),
                            tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0, 'rxyz'),
                            rospy.Time.now(),
                            "marker_11",
                            "world",
        )

        rate.sleep()