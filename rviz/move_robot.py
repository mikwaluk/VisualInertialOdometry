#!/usr/bin/env python3
import turtlesim.msg
import tf
import rospy
import roslib
import numpy as np

if __name__ == '__main__':
    omega = 1
    rospy.init_node('test_tf_broadcaster')
    time = 0
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        time += 0.1
        x = 3 * np.sin(omega * time)
        y = 3 * np.cos(omega * time)
        br.sendTransform((x, y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "world")
        rospy.sleep(0.1)
