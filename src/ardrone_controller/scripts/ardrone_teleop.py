#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_controller.msg import ps3


class ardrone_teleop():
    def __init__(self):
        self.controller_state = ps3()
        self.ps3_sub = rospy.Subscriber('/ps3', ps3, self.ps3_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    
    def ps3_callback(self, msg):
        self.controller_state = msg
    
    def send_command(self):
        current_state = self.controller_state

        cmd = Twist()
        cmd.linear.x = -current_state.axisLY
        cmd.linear.y = -current_state.axisLX
        cmd.angular.z = -current_state.axisRX
        
        #logic for controlling height
        cmd.linear.z = -current_state.trigL2 - -current_state.trigR2

        if(current_state.btnSquare): #land	
            self.land_pub.publish(Empty())
            return
            
        if(current_state.btnTriangle): #takeoff
            self.takeoff_pub.publish(Empty())
            return
            

        self.twist_pub.publish(cmd)



if __name__ == "__main__":
    rospy.init_node("ardrone_teleop")
    teleop = ardrone_teleop()

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        teleop.send_command()
        r.sleep()


