// -------------------- Arduino Headers -----------------------
#include <ArduinoHardware.h>

// ---------------------- ROS Headers -------------------------
#include <ros.h>
#include <std_msgs/UInt16.h>

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
   Callback for /base_move topic, to start moving the car
   forwards or backwards.

   @param mov_msg raw integer data for the case (0 is forwards,
          1 is backwards and otherwise means stop)
*/
void move_callback(const std_msgs::UInt16& mov_msg)
{
  int vel_x = (int) mov_msg.data;

  if (vel_x > 0) //forwards
  {
    analogWrite(ENB, vel_x);
    analogWrite(ENA, vel_x);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(1);
  }
  else if (vel_x < 0) //backwards
  {
    analogWrite(ENB, -vel_x);
    analogWrite(ENA, -vel_x);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(1);
  }
  else //Otherwise stop the motors
  {
    digitalWrite(ENB, LOW);
    digitalWrite(ENA, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    delay(20);
  }
}
// Subscriber to the topic /base_mov
ros::Subscriber<std_msgs::UInt16> sub("/base_mov", &move_callback);

// ----------------------- MAIN PROGRAM -------------------------

void setup()
{
  // Specifying ports previously defined
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize communication
  nh.initNode();
  nh.loginfo("Arduino Setup Completed");
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
}
