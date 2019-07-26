#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "topower_v1/ArmPose.h"
#include "topower_v1/CamPanTilt.h"
#include "topower_v1/WheelDrive.h"

#define PI 3.1415926
#define DEG2RAD PI/180
#define ARM_BASE 0
#define ARM_A 1
#define ARM_B 2
#define GRIPPER_BASE 3
#define GRIPPER 4
#define CAM_PAN 5
#define CAM_TILT 6
#define JOINT_NUM 7

#define WHEEL_L 0
#define WHEEL_R 1
#define WHEEL_NUM 2

class ToPowerV1HW : public hardware_interface::RobotHW{
    public:
        ToPowerV1HW(ros::NodeHandle nh): m_NH(nh){
            for(int i=0;i<JOINT_NUM;i++){
                m_JointPose[i] = 0;
                m_JointVel[i] = 0;
                m_JointEff[i] = 0;
            }
            for(int i=0;i<WHEEL_NUM;i++){
                m_WheelPose[i] = 0;
                m_WheelVel[i] = 0;
                m_WheelEff[i] = 0;
            }
            //position controller: arm, cam pan tilt
            //joint state
            const char* joint[] = {"joint_arm_base","joint_arm_a","joint_arm_b","joint_gripper_base","joint_gripper_l","joint_cam_pan","joint_cam_tilt"};
            for(int i=0;i<JOINT_NUM;i++){
                hardware_interface::JointStateHandle stateHandle(joint[i], &m_JointPose[i], &m_JointVel[i], &m_JointEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //position
            for(int i=0;i<JOINT_NUM;i++){
                hardware_interface::JointHandle posHandle(m_JointStateInterface.getHandle(joint[i]), &m_JointCmd[i]);
                m_PositionJointInterface.registerHandle(posHandle);
            }

            //effort controller: wheel
            //joint state
            const char* wheel[] = {"joint_wheel_l","joint_wheel_r"};
            for(int i=0;i<WHEEL_NUM;i++){
                hardware_interface::JointStateHandle stateHandle(wheel[i], &m_WheelPose[i], &m_WheelVel[i], &m_WheelEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //position
            for(int i=0;i<WHEEL_NUM;i++){
                hardware_interface::JointHandle effortHandle(m_JointStateInterface.getHandle(wheel[i]), &m_WheelCmd[i]);
                m_EffortJointInterface.registerHandle(effortHandle);
            }

            registerInterface(&m_JointStateInterface);
            registerInterface(&m_PositionJointInterface);
            registerInterface(&m_EffortJointInterface);

            LoadOffsetScale();

            m_ArmCmdPub = m_NH.advertise<topower_v1::ArmPose>("/arm_pose_cmd", 1);
            m_ArmPoseSub = m_NH.subscribe("/arm_pose_state", 1, &ToPowerV1HW::ArmPoseCB, this);
            m_CamPanTiltPub = m_NH.advertise<topower_v1::CamPanTilt>("/cam_pan_tilt", 1);
            m_WheelDrivePub = m_NH.advertise<topower_v1::WheelDrive>("/wheel_drive", 1);
        }

        void ArmPoseCB(const topower_v1::ArmPose::ConstPtr& msg){
            //recieve arm pose from esp8266 & pass to joint state controller
            m_JointPose[ARM_BASE] = (msg->armBasePos-m_JointOffset[ARM_BASE])*m_JointScale[ARM_BASE];
            m_JointPose[ARM_A] = (msg->armAPos-m_JointOffset[ARM_A])*m_JointScale[ARM_A];
            m_JointPose[ARM_B] = (msg->armBPos-m_JointOffset[ARM_B])*m_JointScale[ARM_B];
            m_JointPose[GRIPPER_BASE] = (msg->gripperBasePos-m_JointOffset[GRIPPER_BASE])*m_JointScale[GRIPPER_BASE];
            m_JointPose[GRIPPER] = (msg->gripperPos-m_JointOffset[GRIPPER])*m_JointScale[GRIPPER];
            //ROS_INFO("arm pose: %lf %lf %lf %lf %lf %lf", m_JointPose[0],m_JointPose[1],m_JointPose[2],m_JointPose[3],m_JointPose[4],m_JointPose[5]);
        }

        void read(const ros::Time& time, const ros::Duration& period){
            
        }

        void write(const ros::Time& time, const ros::Duration& period){
            //ROS_INFO("arm cmd: %lf %lf %lf %lf %lf %lf", m_JointCmd[0],m_JointCmd[1],m_JointCmd[2],m_JointCmd[3],m_JointCmd[4],m_JointCmd[5]);
            topower_v1::ArmPose armPose;
            armPose.armBasePos = m_JointCmd[ARM_BASE]/m_JointScale[ARM_BASE]+m_JointOffset[ARM_BASE];
            armPose.armAPos = m_JointCmd[ARM_A]/m_JointScale[ARM_A]+m_JointOffset[ARM_A];
            armPose.armBPos = m_JointCmd[ARM_B]/m_JointScale[ARM_B]+m_JointOffset[ARM_B];
            armPose.gripperBasePos = m_JointCmd[GRIPPER_BASE]/m_JointScale[GRIPPER_BASE]+m_JointOffset[GRIPPER_BASE];
            armPose.gripperPos = m_JointCmd[GRIPPER]/m_JointScale[GRIPPER]+m_JointOffset[GRIPPER];
            m_ArmCmdPub.publish(armPose);

            topower_v1::CamPanTilt panTilt;
            panTilt.panPos = m_JointCmd[CAM_PAN]/m_JointScale[CAM_PAN]+m_JointOffset[CAM_PAN];
            panTilt.tiltPos = m_JointCmd[CAM_TILT]/m_JointScale[CAM_TILT]+m_JointOffset[CAM_TILT];
            //ROS_INFO("cmd %d %d", panTilt.panPos, panTilt.tiltPos);
            m_CamPanTiltPub.publish(panTilt);
            //pass cmd to state since we don't have pan tilt feedback
            m_JointPose[CAM_PAN] = m_JointCmd[CAM_PAN];
            m_JointPose[CAM_TILT] = m_JointCmd[CAM_TILT];

            topower_v1::WheelDrive wheelDrive;
            wheelDrive.leftSpeed = m_WheelCmd[WHEEL_L]/m_WheelScale[WHEEL_L]+m_WheelOffset[WHEEL_L];
            wheelDrive.rightSpeed = m_WheelCmd[WHEEL_R]/m_WheelScale[WHEEL_R]+m_WheelOffset[WHEEL_R];
            m_WheelDrivePub.publish(wheelDrive);
            //pass cmd to state since we don't have wheel drive feedback
            m_WheelPose[WHEEL_L] = m_WheelCmd[WHEEL_L];
            m_WheelPose[WHEEL_R] = m_WheelCmd[WHEEL_R];
        }

        void LoadOffsetScale(){
            m_JointOffset[ARM_BASE] = 90;
            m_JointOffset[ARM_A] = 90;
            m_JointOffset[ARM_B] = 90;
            m_JointOffset[GRIPPER_BASE] = 90;
            m_JointOffset[GRIPPER] = 0;
            m_JointOffset[CAM_PAN] = 128;
            m_JointOffset[CAM_TILT] = 128;

            m_JointScale[ARM_BASE] = -DEG2RAD;
            m_JointScale[ARM_A] = DEG2RAD;
            m_JointScale[ARM_B] = DEG2RAD;
            m_JointScale[GRIPPER_BASE] = DEG2RAD;
            m_JointScale[GRIPPER] = -0.0005;
            m_JointScale[CAM_PAN] = 3*PI/(255*4);
            m_JointScale[CAM_TILT] = -PI/(255);

            m_WheelOffset[WHEEL_L] = 0;
            m_WheelOffset[WHEEL_R] = 0;
            m_WheelScale[WHEEL_L] = -0.0007;
            m_WheelScale[WHEEL_R] = -0.0007;
        }

    private:
        ros::NodeHandle m_NH;
        ros::Publisher m_ArmCmdPub,m_CamPanTiltPub,m_WheelDrivePub;
        ros::Subscriber m_ArmPoseSub;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PositionJointInterface m_PositionJointInterface;
        hardware_interface::EffortJointInterface m_EffortJointInterface;
        
        double m_JointCmd[JOINT_NUM];
        double m_JointPose[JOINT_NUM];
        double m_JointVel[JOINT_NUM];
        double m_JointEff[JOINT_NUM];
        double m_JointOffset[JOINT_NUM];
        double m_JointScale[JOINT_NUM];

        double m_WheelCmd[WHEEL_NUM];
        double m_WheelPose[WHEEL_NUM];
        double m_WheelVel[WHEEL_NUM];
        double m_WheelEff[WHEEL_NUM];
        double m_WheelOffset[WHEEL_NUM];
        double m_WheelScale[WHEEL_NUM];
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