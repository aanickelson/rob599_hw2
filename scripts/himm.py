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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# The laser scan message
from sensor_msgs.msg import LaserScan
# Import basic trig functions
from math import sin, cos, pi
import numpy as np
from tf import transformations


class MapMakerMapMakerMakeMeAMap:
    # Find me a find (e.g. a robot that works)
    # Catch me a catch (e.g a robot that works more than once)

    def __init__(self):
        """ Store the map that will be continuously updated"""

        # Initialize occupancy grid
        # Hard coding based on size of my map because finding the map size is not the point of this assignment.
        # Creates 10cm x 10cm grid cells
        self.occ_grid = np.full((100, 100), 0.5)

        # Store the robot's current position
        self.robot_position = (0, 0)
        self.robot_orientation = 0

    def map_update(self, msg):
        """
        Master function that will dictate updates to the map

        :input - laser scan data:
        """

        theta_0 = msg.angle_min
        theta_fin = msg.angle_max
        theta_delta = msg.angle_increment
        iterations = 0

        theta = self.update_theta(self.robot_orientation, theta_0)

        # Loop through all the laser scans
        for distance in msg.ranges:
            # Step through length of scan
            self.update_cells(theta, distance)

            # Update theta for the next laser scan reading
            theta = self.update_theta(theta, theta_delta)

            if theta > theta_fin:
                break

    def update_theta(self, theta, delta):
        """ Update theta and ensure it is within [-pi, pi] """
        theta += delta

        # AMCL_pose converted to radians gives the angle in [-pi, pi]
        # This converts anything that goes above pi or below -pi to the other end of the range
        # There is probably a better way to do this, but.... meh.
        if theta > pi:
            theta = theta - 2 * pi
        elif theta < -pi:
            theta = 2 * pi + theta

        return theta

    def calc_x_y(self, angle, distance):
        """
        TO DO
        Need to do this one tomorrow probably. Because math is hard. Sorry, future Anna.
        """
        x = 1
        y = 1

        return x, y

    def update_cells(self, theta, distance):
        d_curr = 0

        # 10cm discretization in cells
        d_delta = 0.10

        while d_curr < distance:
            x, y = self.calc_x_y(theta, d_curr)

            """ 
            TO DO 
            subtract small amount from that cell
            """

            d_curr += d_delta

        x, y = self.calc_x_y(theta, distance)
        """ 
        TO DO 
        add small amount to that cell
        """

    def set_current_pose(self, msg):
        # Get the current position of the robot and store it
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Mother. Fucking. Quaternions.
        # I remembered ahead of time this time. Didn't have to debug for three hours.
        # But still.... Quaternions.
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.robot_orientation = euler[2]
        print(self.robot_orientation)


if __name__ == '__main__':
    rospy.init_node('himm_py')

    # Charlie the Unicorn!, because this is all make believe.
    Charlie = MapMakerMapMakerMakeMeAMap()

    # A subscriber for the laser scan data
    laser_sub = rospy.Subscriber('scan', LaserScan, Charlie.map_update)
    amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, Charlie.set_current_pose)

    # A publisher for the move data
    rospy.spin()
