#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "topower_v1/ArmPose.h"
#include "topower_v1/CamPanTilt.h"

#define DEG2RAD 3.1415926/180
#define RAD2DEG 180/3.1415926

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
                hardware_interface::JointHandle posHandle(m_JointStateInterface.getHandle(armJoint[i]), &m_ArmCMD[i]);
                m_PositionJointInterface.registerHandle(posHandle);
            }

            //pan tilt
            //joint state
            const char* panTiltJoint[] = {"joint_cam_pan","joint_cam_tilt"};
            for(int i=0;i<2;i++){
                hardware_interface::JointStateHandle stateHandle(panTiltJoint[i], &m_PanTiltPose[i], &m_PanTiltVel[i], &m_PanTiltEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //position
            for(int i=0;i<2;i++){
                hardware_interface::JointHandle posHandle(m_JointStateInterface.getHandle(panTiltJoint[i]), &m_PanTiltCMD[i]);
                m_PositionJointInterface.registerHandle(posHandle);
            }

            //wheel
            //joint state
            const char* wheelJoint[] = {"joint_wheel_l","joint_wheel_r"};
            for(int i=0;i<2;i++){
                hardware_interface::JointStateHandle stateHandle(wheelJoint[i], &m_WheelPose[i], &m_WheelVel[i], &m_WheelEff[i]);
                m_JointStateInterface.registerHandle(stateHandle);
            }
            //effort
            for(int i=0;i<2;i++){
                hardware_interface::JointHandle effortHandle(m_JointStateInterface.getHandle(wheelJoint[i]), &m_WheelCMD[i]);
                m_EffortJointInterface.registerHandle(effortHandle);
            }

            registerInterface(&m_JointStateInterface);
            registerInterface(&m_PositionJointInterface);
            registerInterface(&m_EffortJointInterface);

            LoadOffset();
            ResetArmState();

            m_CmdPub = m_NH.advertise<topower_v1::ArmPose>("arm_pose_cmd", 1);
            m_PoseSub = m_NH.subscribe("/arm_pose_state", 1, &ToPowerV1HW::ArmPoseCB, this);
            m_PanTiltSub = m_NH.subscribe("/pan_tilt_state", 1, &ToPowerV1HW::PanTiltCB, this);
        }

        void PanTiltCB(const topower_v1::CamPanTilt::ConstPtr& msg){
            //recieve arm pose from arduino & pass to joint state controller
            m_PanTiltPose[0] = msg->panPos;
            m_PanTiltPose[1] = msg->tiltPos;
        }

        void ArmPoseCB(const topower_v1::ArmPose::ConstPtr& msg){
            //recieve arm pose from esp8266 & pass to joint state controller
            m_ArmPose[0] = (msg->armBasePos-m_ArmOffset[0])*DEG2RAD;
            m_ArmPose[1] = (msg->armAPos-m_ArmOffset[1])*DEG2RAD;
            m_ArmPose[2] = (msg->armBPos-m_ArmOffset[2])*DEG2RAD;
            m_ArmPose[3] = (msg->gripperBasePos-m_ArmOffset[3])*DEG2RAD;
            m_ArmPose[4] = (msg->gripperPos-m_ArmOffset[4])*-0.0005;
            //ROS_INFO("arm pose: %lf %lf %lf %lf %lf %lf", m_ArmPose[0],m_ArmPose[1],m_ArmPose[2],m_ArmPose[3],m_ArmPose[4],m_ArmPose[5]);
        }

        void read(const ros::Time& time, const ros::Duration& period){
            
        }

        void write(const ros::Time& time, const ros::Duration& period){
            //ROS_INFO("Joint pos: %lf", m_ArmCMD[0]);
        }

        void LoadOffset(){
            m_ArmOffset[0] = 90;
            m_ArmOffset[1] = 90;
            m_ArmOffset[2] = 90;
            m_ArmOffset[3] = 90;
            m_ArmOffset[4] = 0;
        }

        void ResetArmState(){
            for(int i=0;i<5;i++){
                m_ArmPose[i] = 0;
                m_ArmVel[i] = 0;
                m_ArmEff[i] = 0;
            }

            for(int i=0;i<2;i++){
                m_PanTiltPose[i] = 0;
                m_PanTiltVel[i] = 0;
                m_PanTiltEff[i] = 0;
            }

            for(int i=0;i<2;i++){
                m_WheelPose[i] = 0;
                m_WheelVel[i] = 0;
                m_WheelEff[i] = 0;
            }
        }

    private:
        ros::NodeHandle m_NH;
        ros::Publisher m_CmdPub;
        ros::Subscriber m_PoseSub, m_PanTiltSub;

        hardware_interface::JointStateInterface m_JointStateInterface;
        hardware_interface::PositionJointInterface m_PositionJointInterface;
        hardware_interface::EffortJointInterface m_EffortJointInterface;
        
        double m_ArmCMD[5];
        double m_PanTiltCMD[2];
        double m_WheelCMD[2];

        double m_ArmPose[5];
        double m_ArmVel[5];
        double m_ArmEff[5];

        double m_PanTiltPose[2];
        double m_PanTiltVel[2];
        double m_PanTiltEff[2];

        double m_WheelPose[2];
        double m_WheelVel[2];
        double m_WheelEff[2];

        double m_ArmOffset[5];
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