#!/usr/bin/env python

"""
ROB 599 - Spring 2019
Mobile Robotics
Homework 2

Author: Anna Nickelson
"""

# Every python controller needs these lines
import rospy
# The velocity command message
from geometry_msgs.msg import Twist
# The laser scan message
from sensor_msgs.msg import LaserScan
# Import basic trig functions
from math import sin, cos, pi


class MapGenerator:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('himm_py')
    # A subscriber for the laser scan data
    sub = rospy.Subscriber('scan', LaserScan, avoider)
    # A publisher for the move data
    rospy.spin()
