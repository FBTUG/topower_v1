#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyMapper():
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.JoyCallback)
        self.velPub = rospy.Publisher("cmd_vel",Twist,queue_size=1)

    def JoyCallback(self,msg):
        speed = msg.axes[1]
        turnSpeed = -msg.axes[0]
        panOffset = msg.axes[4]
        tiltOffset = msg.axes[5]
        vel = Twist()
        vel.linear.x = speed
        vel.angular.z = turnSpeed
        self.velPub.publish(vel)


if __name__ == '__main__':
    rospy.init_node('joy_mapper_node')
    rospy.loginfo("joy_mapper_node started")
    JoyMapper = JoyMapper()
    rospy.spin()