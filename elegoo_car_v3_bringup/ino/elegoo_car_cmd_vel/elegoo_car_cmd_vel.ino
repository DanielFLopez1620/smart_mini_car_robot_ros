#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// #define USE_USBCON
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

ros::NodeHandle  nh;


void cmd_callback(const geometry_msgs::Twist& mov_msg)
{
  float Setpoint_r, Setpoint_l, x, z;
  bool wtf;
  float w = 0.2;
  int escaling = 220;
  x = mov_msg.linear.x;
  if(x > 1.2)
  {
    x = 1.2;
  }
  if(z > 1.2)
  {
    z = 1.2;
  }
  z = mov_msg.angular.z;
  if(!(x==0 && z==0))
  {
    wtf=false;
    Setpoint_r = x + (z * w / 2.0)/0.1;
    Setpoint_l = x - (z * w / 2.0)/0.1;
  }
  else
  {
    wtf=true;
    Setpoint_l = 0;
    Setpoint_r = 0;
  }
  

  
  if(wtf)
  {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW);  
    digitalWrite(IN3, LOW);  
    digitalWrite(IN4, LOW);
  }
  else
  {
    analogWrite(ENB, Setpoint_l*escaling);
    analogWrite(ENA, Setpoint_r*escaling);
    if(x > 0 && z==0) // Only Forward
    {
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW);  
      digitalWrite(IN3, LOW);  
      digitalWrite(IN4, HIGH);
      nh.loginfo("Forward!");
    }
    else if(x < 0 && z==0)  // Only Backwards
    {
      analogWrite(ENB, Setpoint_l*escaling*0.9);
      analogWrite(ENA, Setpoint_r*escaling*0.9);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      nh.loginfo("Backward!");
    }
    else if(x == 0 && z > 0) // Only Left
    {
      analogWrite(ENA, Setpoint_l*escaling*0.95);
      analogWrite(ENB, Setpoint_r*escaling*0.95);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      nh.loginfo("Left!");
    }
    else if(x == 0 && z <  0) // Only Right
    {
      analogWrite(ENA, Setpoint_l*escaling*0.95);
      analogWrite(ENB, Setpoint_r*escaling*0.95);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      nh.loginfo("Right!");
    }
    else if(x > 0 && z < 0) // Forward and right
    {
      analogWrite(ENB, Setpoint_l*escaling*1.3);
      analogWrite(ENA, Setpoint_r*escaling*-1);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      nh.loginfo("Forward and right!");
    }
    else if(x > 0 && z > 0) // Forward and left
    {
      analogWrite(ENA, Setpoint_l*escaling*1.3);
      analogWrite(ENB, Setpoint_r*escaling*-1);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      nh.loginfo("Forward and left!");
    }
    /*else if(x < 0 && z < 0) // Back and right
    {
      analogWrite(ENA, Setpoint_l*escaling*1.1);
      analogWrite(ENB, Setpoint_r*escaling*-1.3);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      nh.loginfo("Backward and right!");
    }
    else if(x < 0 && z > 0) // Back and left
    {
      analogWrite(ENB, Setpoint_l*200*-1.3);
      analogWrite(ENA, Setpoint_r*200*1.1);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      nh.loginfo("Backward and left!");
    }*/
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
  
}

void loop() 
{
  ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmd_callback);
  nh.subscribe(sub);
  nh.spinOnce();
}
