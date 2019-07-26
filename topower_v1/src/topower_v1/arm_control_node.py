#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from topower_v1.msg import ArmJoyCmd
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import Header
import math

class ArmControl():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.armGroup = moveit_commander.MoveGroupCommander("arm")
        self.planningFrame = self.armGroup.get_planning_frame()
        self.endEffector = self.armGroup.get_end_effector_link()

        self.armJoyCmdSub = rospy.Subscriber("/arm_joy_cmd", ArmJoyCmd, self.ArmJoyCmdCallback)
        self.armTrajectoryCmd = rospy.Publisher("/topower_v1/arm_controller/command", JointTrajectory, queue_size=1)
        #self.trajectoryPublisher = rospy.Publisher('/arm/planned_path',DisplayTrajectory, queue_size=20)
        #rospy.loginfo(self.planningFrame)
        #rospy.loginfo(self.endEffector)
        #rospy.loginfo(self.robot.get_group_names())
        #rospy.loginfo(self.armGroup.get_current_joint_values())

    def ArmJoyCmdCallback(self,msg):
        #rospy.loginfo(msg)
        if msg.resetPose:
            self.GoToStandbyPose()
        elif msg.randomPose:
            self.GoToRandomPose()
        elif msg.armBaseOffset != 0 or msg.armAOffset != 0 or msg.armBOffset != 0 or msg.gripperBaseOffset != 0:
            target = self.armGroup.get_current_joint_values()
            scale = 0.1
            target[0] += msg.armBaseOffset*scale
            target[1] += msg.armAOffset*scale
            target[2] += msg.armBOffset*scale
            target[3] += msg.gripperBaseOffset*scale
            self.armGroup.go(target, wait=True)
            self.armGroup.stop()

        

    def GoToRandomPose(self):
        rospy.loginfo("======go to random pose")
        #pose_goal = self.armGroup.get_random_pose()
        #self.armGroup.set_pose_target(pose_goal)
        #self.armGroup.go(wait=True)

        joint_goal = self.armGroup.get_random_joint_values()
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def GoToStandbyPose(self):
        rospy.loginfo("======go to standby pose")
        joint_goal = self.armGroup.get_named_target_values("standby")
        self.armGroup.go(joint_goal, wait=True)
        self.armGroup.stop()

    def Run(self):
        rospy.spin()
        """rate = rospy.Rate(0.2)
        mode = 0
        while not rospy.is_shutdown():
            rate.sleep()
            if mode % 2 == 0:
                self.GoToRandomPose()
            if mode % 2 == 1:
                self.GoToStandbyPose()
        """

        

if __name__ == '__main__':
    rospy.init_node('arm_control_node')
    rospy.loginfo("arm_control_node started")
    armControl = ArmControl()
    armControl.Run()