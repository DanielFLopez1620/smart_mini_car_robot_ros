// -------------------- Arduino Headers -----------------------
#include <ArduinoHardware.h>

// ---------------------- ROS Headers -------------------------
#include <ros.h>
#include <std_msgs/Empty.h>


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

// ----------------------- FUNCTION DECLARATIONS --------------------

/**
 * Callback for /mov topic, to put on/off mode of the motors.
 * 
 * @param mov Empty message that is used as a flag to change
 *        the state of the motors (toggle)
  */
void motor_callback(const std_msgs::Empty& mov)
{
   digitalWrite(IN1,HIGH-digitalRead(IN1)); 
   digitalWrite(IN2,HIGH-digitalRead(IN2));  
   digitalWrite(IN3,HIGH-digitalRead(IN3));  
   digitalWrite(IN4,HIGH-digitalRead(IN4));
}

// ----------------------- MAIN PROGRAM -------------------------

void setup() 
{
  // Specifying ports previously defined
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);

  // Initialize state of DC Motors
  digitalWrite(ENB,HIGH);
  digitalWrite(ENA,HIGH);
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);
  
  // Initialize communications and subscribe to /mov topic
  nh.initNode();
  ros::Subscriber<std_msgs::Empty> sub("/mov", &motor_callback);
  nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
  delay(1);
}
