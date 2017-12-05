#!/usr/bin/env python

import unittest
import rosunit
import rospy
import rostest
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
PKG = 'velocity_commands'

class TestVelocityCommand(unittest.TestCase):

    def setUp(self):
        self.wait_for_result = False
        self.result = None
        self.twist = TwistStamped()
        rospy.Subscriber("/mcr_manipulation/mcr_arm_cartesian_control/cartesian_velocity_command",TwistStamped,self.callback)

    def test1(self):
        self.assertAlmostEqual(self.twist.twist.linear.x,0)
        self.assertAlmostEqual(self.twist.twist.linear.y,0)
        self.assertAlmostEqual(self.twist.twist.linear.z,0)

    def test2(self):
        self.assertAlmostEqual(self.twist.twist.angular.x,0)
        self.assertAlmostEqual(self.twist.twist.angular.y,0)
        self.assertAlmostEqual(self.twist.twist.angular.z,0)

    def callback(self,msg):
        self.result = msg
        self.wait_for_result = True


    def tearDown(self):
        pass


if __name__ == '__main__':
    rospy.init_node('velocity_commands_node_test')
    rostest.rosrun(PKG, 'velocity_commands_node_test', TestVelocityCommand)
rospy.spin()
