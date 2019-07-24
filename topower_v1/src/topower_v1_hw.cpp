#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "topower_v1/ArmPose.h"
#include "topower_v1/CamPanTilt.h"

#define DEG2RAD 3.1415926/180
#define ARM_BASE 0
#define ARM_A 1
#define ARM_B 2
#define GRIPPER_BASE 3
#define GRIPPER 4
#define CAM_PAN 0
#define CAM_TILT 1

class ToPowerV1HW : public hardware_interface::RobotHW{
    public:
        ToPowerV1HW(ros::NodeHandle nh): m_NH(nh){
            //arm
            //joint state
            const char* armJoint[] = {"joint_arm_base","joint_arm_a","joint_arm_b","joint_gripper_base","joint_gripper_l"};
            for(int i=0;i<5;i++){
                hardware_interface::JointStateHandle stateHandle(armJoint[i], &m_ArmPose[i], &m_ArmVel[i], &m_ArmEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //position
            for(int i=0;i<5;i++){
                hardware_interface::JointHandle posHandle(m_JointStateInterface.getHandle(armJoint[i]), &m_ArmCmd[i]);
                m_PositionJointInterface.registerHandle(posHandle);
            }

            registerInterface(&m_JointStateInterface);
            registerInterface(&m_PositionJointInterface);

            LoadOffsetScale();
            ResetArmState();

            m_ArmCmdPub = m_NH.advertise<topower_v1::ArmPose>("/arm_pose_cmd", 1);
            m_ArmPoseSub = m_NH.subscribe("/arm_pose_state", 1, &ToPowerV1HW::ArmPoseCB, this);
        }

        void ArmPoseCB(const topower_v1::ArmPose::ConstPtr& msg){
            //recieve arm pose from esp8266 & pass to joint state controller
            m_ArmPose[ARM_BASE] = (msg->armBasePos-m_ArmOffset[ARM_BASE])*m_ArmScale[ARM_BASE];
            m_ArmPose[ARM_A] = (msg->armAPos-m_ArmOffset[ARM_A])*m_ArmScale[ARM_A];
            m_ArmPose[ARM_B] = (msg->armBPos-m_ArmOffset[ARM_B])*m_ArmScale[ARM_B];
            m_ArmPose[GRIPPER_BASE] = (msg->gripperBasePos-m_ArmOffset[GRIPPER_BASE])*m_ArmScale[GRIPPER_BASE];
            m_ArmPose[GRIPPER] = (msg->gripperPos-m_ArmOffset[GRIPPER])*m_ArmScale[GRIPPER];
            //ROS_INFO("arm pose: %lf %lf %lf %lf %lf %lf", m_ArmPose[0],m_ArmPose[1],m_ArmPose[2],m_ArmPose[3],m_ArmPose[4],m_ArmPose[5]);
        }

        void read(const ros::Time& time, const ros::Duration& period){
            
        }

        void write(const ros::Time& time, const ros::Duration& period){
            //ROS_INFO("arm cmd: %lf %lf %lf %lf %lf %lf", m_ArmCmd[0],m_ArmCmd[1],m_ArmCmd[2],m_ArmCmd[3],m_ArmCmd[4],m_ArmCmd[5]);
            topower_v1::ArmPose armPose;
            armPose.armBasePos = m_ArmCmd[ARM_BASE]/m_ArmScale[ARM_BASE]+m_ArmOffset[ARM_BASE];
            armPose.armAPos = m_ArmCmd[ARM_A]/m_ArmScale[ARM_A]+m_ArmOffset[ARM_A];
            armPose.armBPos = m_ArmCmd[ARM_B]/m_ArmScale[ARM_B]+m_ArmOffset[ARM_B];
            armPose.gripperBasePos = m_ArmCmd[GRIPPER_BASE]/m_ArmScale[GRIPPER_BASE]+m_ArmOffset[GRIPPER_BASE];
            armPose.gripperPos = m_ArmCmd[GRIPPER]/m_ArmScale[GRIPPER]+m_ArmOffset[GRIPPER];
            m_ArmCmdPub.publish(armPose);
        }

        void LoadOffsetScale(){
            m_ArmOffset[ARM_BASE] = 90;
            m_ArmOffset[ARM_A] = 90;
            m_ArmOffset[ARM_B] = 90;
            m_ArmOffset[GRIPPER_BASE] = 90;
            m_ArmOffset[GRIPPER] = 0;

            m_ArmScale[ARM_BASE] = -DEG2RAD;
            m_ArmScale[ARM_A] = DEG2RAD;
            m_ArmScale[ARM_B] = DEG2RAD;
            m_ArmScale[GRIPPER_BASE] = DEG2RAD;
            m_ArmScale[GRIPPER] = -0.0005;
        }

        void ResetArmState(){
            for(int i=0;i<5;i++){
                m_ArmPose[i] = 0;
                m_ArmVel[i] = 0;
                m_ArmEff[i] = 0;
            }
        }

    private:
        ros::NodeHandle m_NH;
        ros::Publisher m_ArmCmdPub;
        ros::Subscriber m_ArmPoseSub;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PositionJointInterface m_PositionJointInterface;
        
        double m_ArmCmd[5];

        double m_ArmPose[5];
        double m_ArmVel[5];
        double m_ArmEff[5];

        double m_ArmOffset[5];
        double m_ArmScale[5];
};

int main(int argc, char** argv){
    ros::init(argc, argv, "topower_v1_hw");
    ros::NodeHandle nh;

    ToPowerV1HW robot(nh);
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate r(30);
    ros::Time lastTime = ros::Time::now();
    while (ros::ok()){
        ros::Time curTime = ros::Time::now();
        ros::Duration duration = curTime - lastTime;
        //robot.read(curTime,duration);
        cm.update(curTime, duration);
        robot.write(curTime, duration);
        r.sleep();
    }

    return 0;
}