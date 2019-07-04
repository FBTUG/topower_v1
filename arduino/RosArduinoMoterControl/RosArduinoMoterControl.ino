#include <Servo.h>

#include <ros.h>
#include <topower_v1/WheelDrive.h>
#include <topower_v1/PanTiltServo.h>
#include <std_msgs/String.h>
#include <SoftPWM.h>

int LEFT_WHEEL_FORWARD = 11;
int LEFT_WHEEL_BACKWARD = 10;
int RIGHT_WHEEL_FORWARD = 6;
int RIGHT_WHEEL_BACKWARD = 9;

int CAM_PAN = 3;
int CAM_TILT = 5;
Servo panServo, tiltServo;

void ClearPin();
void InitPin();

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher arduinoEcho("arduino_echo", &str_msg);


void wheelDriveCb( const topower_v1::WheelDrive& cmd){
  if(cmd.leftSpeed>0){
    //analogWrite(LEFT_WHEEL_FORWARD,cmd.leftSpeed);
    //analogWrite(LEFT_WHEEL_BACKWARD,0);
    SoftPWMSet(LEFT_WHEEL_FORWARD, cmd.leftSpeed);
    SoftPWMSet(LEFT_WHEEL_BACKWARD, 0);
  }
  else{
    //analogWrite(LEFT_WHEEL_FORWARD,0);
    //analogWrite(LEFT_WHEEL_BACKWARD,-cmd.leftSpeed);
    SoftPWMSet(LEFT_WHEEL_FORWARD, 0);
    SoftPWMSet(LEFT_WHEEL_BACKWARD, -cmd.leftSpeed);
  }

  if(cmd.rightSpeed>0){
    //analogWrite(RIGHT_WHEEL_FORWARD,cmd.rightSpeed);
    //analogWrite(RIGHT_WHEEL_BACKWARD,0);
    SoftPWMSet(RIGHT_WHEEL_FORWARD, cmd.rightSpeed);
    SoftPWMSet(RIGHT_WHEEL_BACKWARD, 0);
  }
  else{
    //analogWrite(RIGHT_WHEEL_FORWARD,0);
    //analogWrite(RIGHT_WHEEL_BACKWARD,-cmd.rightSpeed);
    SoftPWMSet(RIGHT_WHEEL_FORWARD, 0);
    SoftPWMSet(RIGHT_WHEEL_BACKWARD, -cmd.rightSpeed);
  }
  
  String output = "left wheel speed: "+String(cmd.leftSpeed)+" right wheel speed: "+String(cmd.rightSpeed);
  str_msg.data = output.c_str();
  arduinoEcho.publish( &str_msg );
}
ros::Subscriber<topower_v1::WheelDrive> wheelDriveSub("wheel_drive", &wheelDriveCb );

void panTiltServoCb( const topower_v1::PanTiltServo& cmd){
  panServo.write(cmd.panPos);
  tiltServo.write(cmd.tiltPos);

  String output = "pan pos: "+String(cmd.panPos)+" tilt pos: "+String(cmd.tiltPos);
  str_msg.data = output.c_str();
  //arduinoEcho.publish( &str_msg );
}
ros::Subscriber<topower_v1::PanTiltServo> panTiltSub("pan_tilt_servo", &panTiltServoCb );

void ClearPin(){
  
  SoftPWMSet(LEFT_WHEEL_FORWARD, 0);
  SoftPWMSet(LEFT_WHEEL_BACKWARD, 0);
  SoftPWMSet(RIGHT_WHEEL_FORWARD, 0);
  SoftPWMSet(RIGHT_WHEEL_BACKWARD, 0);
  //analogWrite(LEFT_WHEEL_FORWARD,0);
  //analogWrite(LEFT_WHEEL_BACKWARD,0);
  //analogWrite(RIGHT_WHEEL_FORWARD,0);
  //analogWrite(RIGHT_WHEEL_BACKWARD,0);
  panServo.write(128);
  tiltServo.write(128);
}

void InitPin(){
  pinMode(LEFT_WHEEL_FORWARD,OUTPUT);
  pinMode(LEFT_WHEEL_BACKWARD,OUTPUT);
  pinMode(RIGHT_WHEEL_FORWARD,OUTPUT);
  pinMode(RIGHT_WHEEL_BACKWARD,OUTPUT);
  SoftPWMBegin();
  panServo.attach(CAM_PAN);
  tiltServo.attach(CAM_TILT);
  ClearPin();
}

void setup()
{
  nh.initNode();
  nh.advertise(arduinoEcho);
  nh.subscribe(wheelDriveSub);
  nh.subscribe(panTiltSub);
  InitPin();
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
