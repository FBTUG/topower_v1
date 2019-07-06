#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from topower_v1.msg import PanTiltServo

class JoyMapper():
    def __init__(self):
        self.curPan = 128
        self.curTilt = 128
        self.panOffset = 0
        self.tiltOffset = 0
        self.updateRate = rospy.get_param("~updateRate",30)
        self.panMin = rospy.get_param("~panMin",0)
        self.panMax = rospy.get_param("~panMax",255)
        self.tiltMin = rospy.get_param("~tiltMin",0)
        self.tiltMax = rospy.get_param("~tiltMax",255)
        self.sub = rospy.Subscriber("joy", Joy, self.JoyCallback)
        self.velPub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        self.panTiltPub = rospy.Publisher("pan_tilt_servo",PanTiltServo,queue_size=1)
        

    def JoyCallback(self,msg):
        speed = msg.axes[1]
        turnSpeed = -msg.axes[0]
        vel = Twist()
        vel.linear.x = speed
        vel.angular.z = turnSpeed
        self.velPub.publish(vel)

        self.panOffset = msg.axes[4]
        self.tiltOffset = msg.axes[5]
        

    def UpdatePanTilt(self):
        self.curPan += self.panOffset
        self.curTilt += self.tiltOffset
        if self.curPan < self.panMin:
            self.curPan = self.panMin
        elif self.curPan > self.panMax:
            self.curPan = self.panMax
        if self.curTilt < self.tiltMin:
            self.curTilt = self.tiltMin
        elif self.curTilt > self.tiltMax:
            self.curTilt = self.tiltMax
        self.panTiltPub.publish(panPos=self.curPan,tiltPos=self.curTilt)

    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            self.UpdatePanTilt()
            rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('joy_mapper_node')
    rospy.loginfo("joy_mapper_node started")
    JoyMapper = JoyMapper()
    JoyMapper.Run()