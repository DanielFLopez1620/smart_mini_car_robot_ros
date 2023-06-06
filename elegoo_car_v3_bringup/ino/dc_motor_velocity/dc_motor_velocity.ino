// -------------------- Arduino Headers -----------------------
#include <ArduinoHardware.h>

// ---------------------- ROS Headers -------------------------
#include <ros.h>
#include <std_msgs/Int32.h>

// -------------------- Global declarations -------------------

// Ports for H Bridge
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Declare node handle
ros::NodeHandle  nh;

// Velocity considered for the program
int vel_x = 250;

/**
 * Callback to specify a case of direction for the robot, subscribed
 * to the /lineal_move topic.
 * 
 * @param mov_msg Standar message integer, that selects the direction (0:Forward,
 * 1: backwards, 2: left, 3: right, otherwise: stop)
*/
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
}

void setup() 
{
  // Specifying ports previously defined
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  // Initialize communication and subscribe to /linear_move
  nh.initNode();
  ros::Subscriber<std_msgs::Int32> sub("/linear_move", &lineal_callback);
  nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
}
