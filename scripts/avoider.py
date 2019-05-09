#!/usr/bin/env python

"""
ROB 599 - Spring 2019
Mobile Robotics
Written for Homework 1
Adapted for Homework 2

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


def scans_to_vector(msg):
    """
    Takes in the laser scans, does some trig, and returns the resulting x,y vectors in the robot's coordinate frame.
    Robot's coordinate frame will be:
        Forward: positive x
        Backward: negative x
        Left: positive y
        Right: negative y

    This is aligned with the world coordinate frame. If the robot is pointing in the positive x direction, the positive
    y direction will be to its left

    :param msg:
    Laser scan messages
    :return:

    """
    # print(msg.ranges)

    num_scans = len(msg.ranges)
    # Find theta values (in radians)
    theta_0 = msg.angle_min
    theta_fin = msg.angle_max
    theta_delta = msg.angle_increment
    theta_curr = theta_0
    iterations = 0

    # Initialize x and y deltas
    x_delta = 0
    y_delta = 0

    # Loop through all the laser scans
    for distance in msg.ranges:

        # 5.0 is the maximum distance defined. If the distance read is 5.0, then there was likely nothing detected
        # Ignore everything outside the front 90* field of view
        if distance > 3 or iterations < 215 or iterations > 425:
            iterations += 1
            theta_curr += theta_delta
            continue

        # Initialize signs to positive
        y_sign = 1

        # Find the x and y components of the distance vector
        x = abs(cos(theta_curr)) / (num_scans * distance**2)
        y = abs(sin(theta_curr)) / (num_scans * distance**2)

        # If the laser scan is to my left, the resulting y vector should point right (negative)
        if theta_curr > 0:
            y_sign = -1

        # If the laser scan is in front of me, the resulting x vector should point backwards (negative)
        if abs(theta_curr) < pi/2:
            x_sign = -1

        else:
            x_sign = 0.5

        # Add or subtract the inverse square of the resulting distance
        x_delta += x_sign * x
        y_delta += y_sign * y

        # Increment the current angle
        theta_curr += theta_delta
        iterations += 1
        if theta_curr > theta_fin:
            print("Current theta", theta_curr)
            break

    return x_delta, y_delta


def avoider(msg):
    x_vector, y_vector = scans_to_vector(msg)

    command = Twist()
    command.linear.x = 0.2
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0

    if x_vector <= -0.7:
        command.linear.x = -0.5
        if y_vector < 0:
            command.angular.z = -0.25
        else:
            command.angular.z = 0.25

    if x_vector <= -0.5:
        command.linear.x = 0
        if y_vector < 0:
            command.angular.z = -0.25
        else:
            command.angular.z = 0.25
    elif x_vector <= -0.3:
        command.linear.x = 0.05
        if y_vector < 0:
            command.angular.z = -0.2
        else:
            command.angular.z = 0.2

    elif x_vector <= -0.05:
        command.linear.x = 0.15
        if y_vector < 0:
            command.angular.z = -0.15
        else:
            command.angular.z = 0.15

    # Publish the command using the global publisher
    pub.publish(command)


if __name__ == '__main__':
    rospy.init_node('avoid_py')
    # A subscriber for the laser scan data
    sub = rospy.Subscriber('scan', LaserScan, avoider)
    # A publisher for the move data
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    rospy.spin()
