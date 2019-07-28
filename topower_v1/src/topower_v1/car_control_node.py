#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from topower_v1.msg import WheelDrive, CamPanTilt
from std_msgs.msg import Float64
import math

class CarControl():
    def __init__(self):
        self.leftRatio = rospy.get_param("~leftRatio",1)
        self.rightRatio = rospy.get_param("~rightRatio",1)
        self.turnRatio = rospy.get_param("~turnRatio",1)
        self.minPWM = rospy.get_param("~minPWM",60)
        self.maxPWM = rospy.get_param("~maxPWM",255)
        self.minSpeed = rospy.get_param("~minSpeed",0.1)
        self.speed = 0  #range = -1~1
        self.turnSpeed = 0  #range = -1~1
        rospy.loginfo("use leftRatio=%f rightRatio=%f minPWM=%d maxPWM=%d minSpeed=%f" % (self.leftRatio,self.rightRatio,self.minPWM,self.maxPWM,self.minSpeed))

        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.VelocityCallback)
        self.wheelLPub = rospy.Publisher("/topower_v1/wheel_l_effort_controller/command",Float64,queue_size=1)
        self.wheelRPub = rospy.Publisher("/topower_v1/wheel_r_effort_controller/command",Float64,queue_size=1)

    def VelocityCallback(self,msg):
        #twist.linear: x->forward, y->left, z->up
        #twist.angular:rotate about axis
        if math.fabs(msg.linear.x) > 1:
            raise ValueError('Speed must be between -1 and 1')
        if math.fabs(msg.angular.z) > 1:
            raise ValueError('turn speed must be between -1 and 1')

        self.speed = -msg.linear.x
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
        leftSpeed = self.leftRatio*0.5*(self.speed-self.turnSpeed*self.turnRatio)
        rightSpeed = self.rightRatio*0.5*(self.speed+self.turnSpeed*self.turnRatio)
        leftPWM = self.ComputePWM(leftSpeed)
        rightPWM = self.ComputePWM(rightSpeed)
        scale = 0.2/255.0
        self.wheelLPub.publish(scale*leftPWM)
        self.wheelRPub.publish(scale*rightPWM)


if __name__ == '__main__':
    rospy.init_node('car_control_node')
    rospy.loginfo("car_control_node started")
    carControl = CarControl()
    rospy.spin()