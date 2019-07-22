
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
ros::Publisher armPosePub("arm_pose_state", &armPoseState);

void CommandCB( const topower_v1::ArmPose& armPose){
  servo.move(ARM_BASE, armPose.armBasePos, 128);
  servo.move(ARM_A, armPose.armAPos, 128);
  servo.move(ARM_B, armPose.armBPos, 128);
  servo.move(GRIPPER_BASE, armPose.gripperBasePos, 128);
  servo.move(GRIPPER, armPose.gripperPos, 128);
}
ros::Subscriber<topower_v1::ArmPose> sub("arm_pose_cmd", &CommandCB);

void PublishState(){
  armPoseState.armBasePos = servo.getPos(ARM_BASE);
  armPoseState.armAPos = servo.getPos(ARM_A);
  armPoseState.armBPos = servo.getPos(ARM_B);
  armPoseState.gripperBasePos = servo.getPos(GRIPPER_BASE);
  armPoseState.gripperPos = servo.getPos(GRIPPER);
  armPosePub.publish(&armPoseState);
}

void setup()
{
  // Delay 2s to wait for all servo started
  delay(2000);

  Serial.begin(115200);
  Serial.println();
  
  servo.init(MOTOR_NUM);
  servo.setDebug(false);
  retBuffer = servo.retBuffer();
  servo.begin();
  //servo.lockAll();
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
}



void loop()
{
  PublishState();
  nh.spinOnce();
  delay(10);
}
