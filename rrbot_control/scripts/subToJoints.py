#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def JointStatesCallback(data):

    q = data.position  # class tuple, in radians
    qDot = data.velocity  # in rad/s

    print('joint pose:', q)


############
# main loop:
############

if __name__ == '__main__':

    rospy.init_node('subToJoints_node')

    try:
        rospy.Subscriber("/rrbot/joint_states", JointState, JointStatesCallback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
