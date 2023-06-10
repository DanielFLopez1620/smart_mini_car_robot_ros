// ------------------------ Arduino Headers ----------------------
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Servo.h>

// ------------------------ ROS Headers -------------------------
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>

// ------------------------ Global declarations ------------------
ros::NodeHandle  nh;
Servo servo;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

int pingPin = A5; // trigger 
int inPin = A4; // echo
long range_time;
char frameid[] = "/ultrasound";



// ----------------------- FUNCTIONS DECLARATIONS -------------------------

/**
 * Servo callback to move the servo to a given position
 * 
 * @param cmd_msg Standard message of type unsigned int16 which
 * contains the desired angular position. 
 */
void servo_cb( const std_msgs::UInt16& cmd_msg)
{
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}
ros::Subscriber<std_msgs::UInt16> sub("/servo", servo_cb);

/**
 * Convert the microseconds that lasted the ultrasonic pulse to an
 * estimation of the distance.
 * 
 * @param microseconds Time percieved by the ultrasonic sensor.
 * 
 * @return centimeters Distance estimation
  */
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29.1 / 2;
}

/**
 * Reads the duration of a pulse sent by a ultrasonic sensor and
 * converts it to a estimation of distance.
 */
float getRange()
{
  long duration, cm;
  
  // Using the ultrasonic sensor to send a pulse
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  // Reading the echo's pulse duration
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

// ----------------------- MAIN PROGRAM -------------------------
void setup() 
{
  // Initialize communication
  nh.initNode();
  
  // Begin specifications for ultrasound publisher
  nh.advertise(pub_range);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;

  // Subscribe to servo indicator
  nh.subscribe(sub);
  servo.attach(3);
}

void loop()
{
  if ( millis() >= range_time ){
      int r =0;  
      range_msg.range = getRange() / 100;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
      range_time =  millis() + 50;
    }    
    nh.spinOnce();
}
