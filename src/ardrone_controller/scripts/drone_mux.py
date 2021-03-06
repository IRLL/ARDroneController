#!/usr/bin/env python

"""
Author: James Irwin (james.irwin@wsu.edu)
Description:
    Accepts twist messages to forward to quadcopter, ensuring
    that the quadrotor stays within specified bounds.
"""

import rospy
from sensor_msgs.msg import Image #for recieving video feed
from geometry_msgs.msg import Twist # controlling the movements
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty #send empty message for takeoff and landing
import tf
from math import sin,cos

class DroneMux:
    def __init__(self):
        self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleop_callback)
        self.location_sub = rospy.Subscriber('/ardrone/location', Pose, self.location_callback)
        #create message publisher for sending drone movement commands
        self.output_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.location = Pose()
        self.bounds = rospy.get_param("/bounds")

    def teleop_callback(self, twist):
        #check if we're within bounds
        if self.in_bounds():
            output = twist
        else: #we need to correct
            output = self.get_correction()

        self.output_pub.publish(output)

    def in_bounds(self):
        if (self.location.position.x < self.bounds['x']['max'] and
            self.location.position.y < self.bounds['y']['max'] and
            self.location.position.z < self.bounds['z']['max'] and
            self.location.position.x > self.bounds['x']['min'] and
            self.location.position.y > self.bounds['y']['min'] and
            self.location.position.z > self.bounds['z']['min']):
            return True
        else:
            return False

    def get_correction(self):
        twist = Twist()
        correction_speed = 0.5
        #check for what bounds need to be fixed in absolute frame
        #X
        x = 0
        y = 0
        if(self.location.position.x < self.bounds['x']['min']):
            x = correction_speed
        if(self.location.position.x > self.bounds['x']['max']):
            x = -correction_speed
        #Y
        if(self.location.position.y < self.bounds['y']['min']):
            y = correction_speed
        if(self.location.position.y > self.bounds['y']['max']):
            y = -correction_speed
        #Z
        if(self.location.position.z < self.bounds['z']['min']):
            twist.linear.z = correction_speed
        if(self.location.position.z > self.bounds['z']['max']):
            twist.linear.z = -correction_speed

        #now to to transform values to ardrone's frame of reference
        #no need to convert z, only x,y
        (_, _, theta) = tf.transformations.euler_from_quaternion([
            self.location.orientation.x,
            self.location.orientation.y,
            self.location.orientation.z,
            self.location.orientation.w])
        twist.linear.x = x*cos(theta) + y*sin(theta)
        twist.linear.y = -x*sin(theta) + y*cos(theta)

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

