#!/usr/bin/env python

"""
Author: James Irwin (james.irwin@wsu.edu)
Description:
    Publishes Gazebo pose information to /ardrone/location
"""

import rospy
from geometry_msgs.msg import Pose,PoseStamped
from gazebo_msgs.msg import ModelStates

class Location:
    def __init__(self):
        self.location_sub = rospy.Subscriber('/vrpn_client_node/ardrone/pose', PoseStamped, self.location_callback)
        #create message publisher for sending drone movement commands
        self.output_pub = rospy.Publisher('/ardrone/location', Pose, queue_size=1)
        self.count = 0

    def location_callback(self, location):
        #gazebo publishes at 100hz, only publish every 5th message to get 20hz
        self.count += 1
        if self.count > 4:
            quadrotor_pose = location.pose
            self.output_pub.publish(quadrotor_pose)
            self.count = 0


if __name__ == "__main__":
    rospy.init_node("location")

    location = Location()

    #this function loops and waits for node to shutdown
    #all logic happens in the image_callback function
    rospy.spin()
