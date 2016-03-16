#!/usr/bin/env python

"""
Author: James Irwin (james.irwin@wsu.edu)
Description:
	Example code for getting started with the ardrone
"""

import rospy
from sensor_msgs.msg import Image #for recieving video feed
from geometry_msgs.msg import Twist # controlling the movements
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty #send empty message for takeoff and landing

class DroneMux:
	def __init__(self):
		self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleop_callback)
		self.location_sub = rospy.Subscriber('/ardrone/location', Pose, self.location_callback)
		#create message publisher for sending drone movement commands
		self.output_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.location = Pose()

	def teleop_callback(self, twist):
		#check if we're within bounds
		if self.in_bounds():
			print 'yes'
			output = twist
		else: #we need to correct
			print 'no'
		 	output = self.get_correction()

		self.output_pub.publish(output)

	def in_bounds(self):
		print self.location.position.z
		if self.location.position.z < 3:
			return True
		else:
			return False

	def get_correction(self):
		twist = Twist()
		twist.linear.z = -0.05
		return twist

	
	def location_callback(self, location):
		self.location = location
	

#callback function that gets run when node shuts down
def shutdown_callback():
	print "shutting down..."
	drone_land_pub.publish(Empty()) #make the drone land
	print "done!"


if __name__ == "__main__":
	rospy.init_node("drone_mux")
	#ardrone uses specialized topics for landing and taking off
	drone_takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
	drone_land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
	mux = DroneMux()

	#register callback function to land drone if we kill the program
	rospy.on_shutdown(shutdown_callback) 

	rospy.sleep(1) #wait for a second to wait for node to fully connect
	drone_takeoff_pub.publish(Empty()) #command drone to takeoff

	#this function loops and waits for node to shutdown
	#all logic happens in the image_callback function
	rospy.spin()

