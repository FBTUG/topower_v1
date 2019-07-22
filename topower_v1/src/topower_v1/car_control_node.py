#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from topower_v1.msg import WheelDrive, CamPanTilt
import math

class CarControl():
    def __init__(self):
        self.turnRatio = rospy.get_param("~turnRatio",1)
        self.minPWM = rospy.get_param("~minPWM",60)
        self.maxPWM = rospy.get_param("~maxPWM",255)
        self.minSpeed = rospy.get_param("~minSpeed",0.1)
        self.speed = 0  #range = -1~1
        self.turnSpeed = 0  #range = -1~1
        self.vnorm = 1/(1+0.5*self.turnRatio)
        rospy.loginfo("use turnRatio=%f minPWM=%d maxPWM=%d minSpeed=%f" % (self.turnRatio,self.minPWM,self.maxPWM,self.minSpeed))

        self.sub = rospy.Subscriber("cmd_vel", Twist, self.VelocityCallback)
        self.wheelDrivePub = rospy.Publisher("wheel_drive",WheelDrive,queue_size=1)

    def VelocityCallback(self,msg):
        #twist.linear: x->forward, y->left, z->up
        #twist.angular:rotate about axis
        if math.fabs(msg.linear.x) > 1:
            raise ValueError('Speed must be between -1 and 1')
        if math.fabs(msg.angular.z) > 1:
            raise ValueError('turn speed must be between -1 and 1')

        self.speed = msg.linear.x
        self.turnSpeed = msg.angular.z
        self.UpdateVelocity()

    def ComputePWM(self,v):
        absV = math.fabs(v)
        if absV < self.minSpeed:
            return 0
        else:
            if v > 0:
                return int(absV*(self.maxPWM-self.minPWM)+self.minPWM)
            else:
                return -int(absV*(self.maxPWM-self.minPWM)+self.minPWM)

    def UpdateVelocity(self):
        leftSpeed = self.speed*self.vnorm*(1-self.turnSpeed*0.5*self.turnRatio)
        rightSpeed = self.speed*self.vnorm*(1+self.turnSpeed*0.5*self.turnRatio)
        wheelDrive = WheelDrive()
        wheelDrive.leftSpeed = self.ComputePWM(leftSpeed)
        wheelDrive.rightSpeed = self.ComputePWM(rightSpeed)
        self.wheelDrivePub.publish(wheelDrive)


if __name__ == '__main__':
    rospy.init_node('car_control_node')
    rospy.loginfo("car_control_node started")
    carControl = CarControl()
    rospy.spin()