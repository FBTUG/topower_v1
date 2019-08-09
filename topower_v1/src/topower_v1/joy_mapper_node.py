#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from topower_v1.msg import CamPanTilt,ArmJoyCmd
from std_msgs.msg import Float64,Empty
import math

class JoyMapper():
    def __init__(self):
        self.curPan = 0
        self.curTilt = 0
        self.panOffset = 0
        self.tiltOffset = 0

        self.armBaseOffset = 0
        self.armAOffset = 0
        self.armBOffset = 0
        self.gripperBaseOffset = 0
        self.resetPose = False
        self.randomPose = False
        self.mode = "car"

        self.curGripperPos = 0
        self.gripperPosOffset = 0

        self.updateRate = rospy.get_param("~updateRate",30)

        self.panMin = rospy.get_param("~panMin",-2*math.pi/3)
        self.panMax = rospy.get_param("~panMax",2*math.pi/3)
        self.tiltMin = rospy.get_param("~tiltMin",-0.5*math.pi)
        self.tiltMax = rospy.get_param("~tiltMax",0.5*math.pi)
        self.gripperPosMin = rospy.get_param("~gripperPosMin",-0.035)
        self.gripperPosMax = rospy.get_param("~gripperPosMax",0)

        self.armBaseMin = rospy.get_param("~armBaseMin",0)
        self.armBaseMax = rospy.get_param("~armBaseMax",180)
        self.armAMin = rospy.get_param("~armAMin",0)
        self.armAMax = rospy.get_param("~armAMax",180)
        self.armBMin = rospy.get_param("~armBMin",0)
        self.armBMax = rospy.get_param("~armBMax",180)
        self.gripperBaseMin = rospy.get_param("~gripperBaseMin",0)
        self.gripperBaseMax = rospy.get_param("~gripperBaseMax",180)
        self.gripperMin = rospy.get_param("~gripperMin",0)
        self.gripperMax = rospy.get_param("~gripperMax",120)

        self.leftStickX = rospy.get_param("~leftStickX",0)
        self.leftStickY = rospy.get_param("~leftStickY",1)
        self.rightStickX = rospy.get_param("~rightStickX",3)
        self.rightStickY = rospy.get_param("~rightStickY",4)
        self.axisLT = rospy.get_param("~axisLT",2)
        self.axisRT = rospy.get_param("~axisRT",5)
        self.dirX = rospy.get_param("~dirX",6)
        self.dirY = rospy.get_param("~dirY",7)

        self.btA = rospy.get_param("~btA",0)
        self.btB = rospy.get_param("~btB",1)
        self.btX = rospy.get_param("~btX",2)
        self.btY = rospy.get_param("~btY",3)
        self.btLB = rospy.get_param("~btLB",4)
        self.btRB = rospy.get_param("~btRB",5)
        self.btBack = rospy.get_param("~btBack",6)
        self.btStart = rospy.get_param("~btStart",7)

        self.sub = rospy.Subscriber("joy", Joy, self.JoyCallback)
        self.velPub = rospy.Publisher("cmd_vel",Twist,queue_size=1)
        self.camPanPub = rospy.Publisher("/topower_v1/cam_pan_position_controller/command",Float64,queue_size=1)
        self.camTiltPub = rospy.Publisher("/topower_v1/cam_tilt_position_controller/command",Float64,queue_size=1)
        self.gripperPosPub = rospy.Publisher("/topower_v1/gripper_l_position_controller/command",Float64,queue_size=1)
        self.armJoyCmdPub = rospy.Publisher("arm_joy_cmd",ArmJoyCmd,queue_size=1)
        self.camSavePub = rospy.Publisher("/topower_v1/camera/capture",Empty,queue_size=1)

    def JoyCallback(self,msg):
        if self.mode == "car":
            speed = msg.axes[self.leftStickY]
            turnSpeed = -msg.axes[self.leftStickX]
            vel = Twist()
            vel.linear.x = speed
            vel.angular.z = turnSpeed
            self.velPub.publish(vel)

            self.panOffset = msg.axes[self.rightStickX]
            self.tiltOffset = -msg.axes[self.rightStickY]

            if msg.buttons[self.btY] == 1:  #reset pan tilt pose
                self.curPan = 0
                self.curTilt = 0
            if msg.buttons[self.btX] == 1:
                self.mode = "arm"
                rospy.loginfo("change to arm mode")
            if msg.buttons[self.btA == 1]:  #trigger image save
                self.camSavePub.publish()

        elif self.mode == "arm":
            self.armBaseOffset = msg.axes[self.dirX]
            self.gripperBaseOffset = msg.axes[self.dirY]
            self.armAOffset = msg.axes[self.leftStickY]
            self.armBOffset = msg.axes[self.rightStickY]

            if msg.buttons[self.btA] == 1:
                self.curGripperPos = self.gripperMin
            elif msg.buttons[self.btB] == 1:
                self.curGripperPos = self.gripperMax
            
            if msg.buttons[self.btY] == 1:
                cmd = ArmJoyCmd()
                cmd.randomPose = True
                self.armJoyCmdPub.publish(cmd)

            if msg.buttons[self.btStart] == 1:
                cmd = ArmJoyCmd()
                cmd.resetPose = True
                self.armJoyCmdPub.publish(cmd)

            if msg.buttons[self.btX] == 1:
                self.mode = "car"
                rospy.loginfo("change to car mode")
        

    def Update(self):
        scale = 0.1
        self.curPan += self.panOffset*scale
        self.curTilt += self.tiltOffset*scale
        if self.curPan < self.panMin:
            self.curPan = self.panMin
        elif self.curPan > self.panMax:
            self.curPan = self.panMax
        if self.curTilt < self.tiltMin:
            self.curTilt = self.tiltMin
        elif self.curTilt > self.tiltMax:
            self.curTilt = self.tiltMax

        self.camPanPub.publish(self.curPan)
        self.camTiltPub.publish(self.curTilt)

        
        cmd = ArmJoyCmd()
        cmd.armBaseOffset = self.armBaseOffset
        cmd.armAOffset = self.armAOffset
        cmd.armBOffset = self.armBOffset
        cmd.gripperBaseOffset = self.gripperBaseOffset
        cmd.resetPose = False
        cmd.randomPose = False
        self.armJoyCmdPub.publish(cmd)

        self.gripperPosPub.publish(self.curGripperPos)


    def Run(self):
        rate = rospy.Rate(self.updateRate)
        while not rospy.is_shutdown():
            self.Update()
            rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('joy_mapper_node')
    rospy.loginfo("joy_mapper_node started")
    JoyMapper = JoyMapper()
    JoyMapper.Run()