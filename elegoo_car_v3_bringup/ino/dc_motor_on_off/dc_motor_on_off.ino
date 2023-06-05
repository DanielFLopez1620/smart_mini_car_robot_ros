#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Empty.h>


// #define USE_USBCON
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

ros::NodeHandle  nh;
float vel_linear = 0;

void motor_callback(const std_msgs::Empty& mov)
{
   digitalWrite(IN1,HIGH-digitalRead(IN1)); 
   digitalWrite(IN2,HIGH-digitalRead(IN2));  
   digitalWrite(IN3,HIGH-digitalRead(IN3));  
   digitalWrite(IN4,HIGH-digitalRead(IN4));
   return;
}

ros::Subscriber<std_msgs::Empty> sub("mov", &motor_callback);

void setup() 
{
  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  digitalWrite(ENB,HIGH);
  digitalWrite(ENA,HIGH);
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
  delay(1);
}
