/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

const char* ssid = "aga-mobile";
const char* password = "aga-wifi";
IPAddress server(192,168,43,237);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411; // Set the rosserial socket server port

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(ssid, password);

  Serial.print("Connecting");
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
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
