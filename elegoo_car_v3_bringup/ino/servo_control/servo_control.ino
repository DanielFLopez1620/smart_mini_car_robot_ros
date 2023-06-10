// ---------------------- Arduino headers --------------------------
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Servo.h>
// ---------------------- ROS Headers ------------------------------
 
#include <ros.h>
#include <std_msgs/UInt16.h>

// ---------------------- Global declarations -----------------------
ros::NodeHandle  nh;
Servo servo;

// ----------------------- FUNCTION DECLARATIONS --------------------

/**
 * Servo callback to move the servo to a given position
 * 
 * @param cmd_msg Standard message of type unsigned int16 which
 * contains the desired angular position. 
 */
void servo_cb( const std_msgs::UInt16& cmd_msg)
{
  servo.write(cmd_msg.data); //set servo angle 
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

// Definition of the subscriber
ros::Subscriber<std_msgs::UInt16> sub("/servo", servo_cb);


// ----------------------- MAIN PROGRAM -------------------------
void setup(){
  pinMode(13, OUTPUT);

  // Begin communication and subscribe to /servo
  nh.initNode();
  nh.subscribe(sub);
  
  //attach pin to servo
  servo.attach(3); 
}

void loop(){
  nh.spinOnce();
  delay(1);
}
