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
int vel_x = 250;

void lineal_callback(const std_msgs::Int32& mov_msg)
{
  int case_mov = mov_msg.data;
  if(case_mov == 0) //forwards
  {
    analogWrite(ENB,vel_x);
    analogWrite(ENA,vel_x);
    digitalWrite(IN1,HIGH); 
    digitalWrite(IN2,LOW);  
    digitalWrite(IN3,LOW);  
    digitalWrite(IN4,HIGH);
    delay(1);
  }
  else if(case_mov == 1) //backwards
  {
    analogWrite(ENB,vel_x);
    analogWrite(ENA,vel_x);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    delay(1);
  }
  else if (case_mov == 2) //left
  {
    analogWrite(ENB, vel_x);
    analogWrite(ENA, vel_x);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (case_mov == 3) //right
  {
    analogWrite(ENA, vel_x);
    analogWrite(ENB, vel_x);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
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
  ros::Subscriber<std_msgs::Int32> sub("/linear_move", &lineal_callback);
  nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
  //nh.loginfo("SpinOnce");
}
