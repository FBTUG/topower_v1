
#include <ESP8266WiFi.h>
#include <ros.h>
#include <topower_v1/ArmPose.h>
#include "UBTech.h"

const char* ssid = "aga-mobile";
const char* password = "aga-wifi";
IPAddress server(192,168,43,237);
const uint16_t serverPort = 11411;


SoftwareSerial ss12(12, 12, false, 256);
UBTech servo(&ss12, &Serial);
byte *retBuffer;
#define MOTOR_NUM 5
#define ARM_BASE 1
#define ARM_A 2
#define ARM_B 3
#define GRIPPER_BASE 4
#define GRIPPER 5

ros::NodeHandle  nh;
topower_v1::ArmPose armPoseCmd, armPoseState;
ros::Publisher armPosePub("/arm_pose_state", &armPoseState);

void CommandCB( const topower_v1::ArmPose& armPose){
  armPoseCmd.armBasePos = armPose.armBasePos;
  armPoseCmd.armAPos = armPose.armAPos;
  armPoseCmd.armBPos = armPose.armBPos;
  armPoseCmd.gripperBasePos = armPose.gripperBasePos;
  armPoseCmd.gripperPos = armPose.gripperPos;
}
ros::Subscriber<topower_v1::ArmPose> armCmdSub("/arm_pose_cmd", &CommandCB);

void PublishState(){
  armPoseState.armBasePos = servo.getPos(ARM_BASE);
  if(armPoseState.armBasePos == 255){
    armPoseState.armBasePos = servo.lastAngle(ARM_BASE);  
  }
  
  armPoseState.armAPos = servo.getPos(ARM_A);
  if(armPoseState.armAPos == 255){
    armPoseState.armAPos = servo.lastAngle(ARM_A);  
  }
  
  armPoseState.armBPos = servo.getPos(ARM_B);
  if(armPoseState.armBPos == 255){
    armPoseState.armBPos = servo.lastAngle(ARM_B);  
  }
  
  armPoseState.gripperBasePos = servo.getPos(GRIPPER_BASE);
  if(armPoseState.gripperBasePos == 255){
    armPoseState.gripperBasePos = servo.lastAngle(GRIPPER_BASE);  
  }
  
  armPoseState.gripperPos = servo.getPos(GRIPPER);
  if(armPoseState.gripperPos == 255){
    armPoseState.gripperPos = servo.lastAngle(GRIPPER);  
  }
  
  armPosePub.publish(&armPoseState);
}

void MoveArm(){
  //compute move speed
  int armBaseDiff = abs(armPoseCmd.armBasePos-armPoseState.armBasePos);
  int armADiff = abs(armPoseCmd.armAPos-armPoseState.armAPos);
  int armBDiff = abs(armPoseCmd.armBPos-armPoseState.armBPos);
  int gripperBaseDiff = abs(armPoseCmd.gripperBasePos-armPoseState.gripperBasePos);
  int gripperDiff = abs(armPoseCmd.gripperPos-armPoseState.gripperPos);
  //set speed thresh to reduce jitter
  int minThresh = 30;
  if(armBaseDiff < minThresh) armBaseDiff = minThresh;
  if(armADiff < minThresh) armADiff = minThresh;
  if(armBDiff < minThresh) armBDiff = minThresh;
  if(gripperBaseDiff < minThresh) gripperBaseDiff = minThresh;
  if(gripperDiff < minThresh) gripperDiff = minThresh;
  
  //move arm
  servo.move(ARM_BASE, armPoseCmd.armBasePos, armBaseDiff);
  servo.move(ARM_A, armPoseCmd.armAPos, armADiff);
  servo.move(ARM_B, armPoseCmd.armBPos, armBDiff);
  servo.move(GRIPPER_BASE, armPoseCmd.gripperBasePos, gripperBaseDiff);
  servo.move(GRIPPER, armPoseCmd.gripperPos, gripperDiff);
}

void setup()
{
  // Delay 2s to wait for all servo started
  delay(2000);

  Serial.begin(115200);
  Serial.println();

  armPoseCmd.armBasePos = 90;
  armPoseCmd.armAPos = 90;
  armPoseCmd.armBPos = 90;
  armPoseCmd.gripperBasePos = 90;
  armPoseCmd.gripperPos = 0;
  
  servo.init(MOTOR_NUM);
  servo.setDebug(false);
  retBuffer = servo.retBuffer();
  servo.begin();
  servo.setLED(0, 0);
  
  

  WiFi.begin(ssid, password);

  //Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(armPosePub);
  nh.subscribe(armCmdSub);
}

void loop()
{
  PublishState();
  delay(1);
  MoveArm();
  nh.spinOnce();
}
