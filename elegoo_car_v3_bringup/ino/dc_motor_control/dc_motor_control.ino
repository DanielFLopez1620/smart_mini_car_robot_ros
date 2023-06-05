#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int32.h>

// #define USE_USBCON
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

ros::NodeHandle  nh;
float vel_linear = 0;

void move_callback(const std_msgs::Int32& mov_msg)
{
  int vel_x = mov_msg.data;
  // nh.loginfo("Arrived");

  if(vel_x > 0) //forwards
  {
    analogWrite(ENB,vel_x);
    analogWrite(ENA,vel_x);
    digitalWrite(IN1,HIGH); 
    digitalWrite(IN2,LOW);  
    digitalWrite(IN3,LOW);  
    digitalWrite(IN4,HIGH);
    delay(1);
  }
  else if(vel_x < 0) //backwards
  {
    analogWrite(ENB,-vel_x);
    analogWrite(ENA,-vel_x);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    delay(1);
  }
  else //Otherwise stop the motors
  {
    digitalWrite(ENB,LOW);  
    digitalWrite(ENA,LOW);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    delay(20);
  }
  return;
}

void setup() 
{
  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  nh.initNode();
  ros::Subscriber<std_msgs::Int32> sub("/base_mov", &move_callback);
  nh.subscribe(sub);

  nh.spinOnce();
  nh.logwarn("Arduino Setup Completed");
  nh.spinOnce();
}

void loop() 
{
  nh.spinOnce();
  //nh.loginfo("SpinOnce");
}
