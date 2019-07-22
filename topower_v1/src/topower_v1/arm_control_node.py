#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
import math

class ArmControl():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.armGroup = moveit_commander.MoveGroupCommander("arm")
        self.planningFrame = self.armGroup.get_planning_frame()
        self.endEffector = self.armGroup.get_end_effector_link()

        #self.trajectoryPublisher = rospy.Publisher('/arm/planned_path',DisplayTrajectory, queue_size=20)
        #rospy.loginfo(self.planningFrame)
        #rospy.loginfo(self.endEffector)
        #rospy.loginfo(self.robot.get_group_names())
        #rospy.loginfo(self.armGroup.get_current_joint_values())

    def GoToRandomPose(self):
        rospy.loginfo("======go to random pose")
        joint_goal = self.armGroup.get_random_joint_values()
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def GoToStandbyPose(self):
        rospy.loginfo("======go to standby pose")
        joint_goal = self.armGroup.get_named_target_values("standby")
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def Run(self):
        rate = rospy.Rate(0.2)
        mode = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if mode % 2 == 0:
                self.GoToRandomPose()
            if mode % 2 == 1:
                self.GoToStandbyPose()

        

if __name__ == '__main__':
    rospy.init_node('arm_control_node')
    rospy.loginfo("arm_control_node started")
    armControl = ArmControl()
    armControl.Run()