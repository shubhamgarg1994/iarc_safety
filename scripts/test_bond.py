#!/usr/bin/env python
import rospy
from iarc7_safety.SafetyClient import SafetyClient

if __name__ == '__main__':
    rospy.init_node('test_bond')

    rospy.loginfo('Trying to form bond')
    safety_client = SafetyClient('test_bond')
    assert(safety_client.form_bond())
    rospy.loginfo('Bond formed')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Exit immediately if fatal
        if safety_client.is_fatal_active():
            rospy.loginfo('Fatal active')
            break;
        # Exit if safety state, this node can't do anything about it
        if safety_client.is_safety_active():
            rospy.loginfo('Safety active')
            break;
        rate.sleep()
