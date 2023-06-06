// -------------------- Arduino Headers -----------------------
#include <Servo.h> 
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#define USE_USBCON
#include <ArduinoHardware.h>

// -------------------- ROS headers -------------------
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>

// -------------------- Global declarations -------------------

// Ports for H Bridge
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Define node handle
ros::NodeHandle  nh;

// Declare sensor_msg and publisher for the ultrasonic sensor.
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

// Analog port for the ultrasonic sensor.
int pingPin = A5; // trigger 
int inPin = A4; // echo

// Declaration of properites for the sensor_msg
long range_time;
char frameid[] = "/ultrasound";

// Velocity considered for the motors.
int vel_x = 250;

// Servo Declaration
Servo servo;

// ----------------------- FUNCTION DECLARATIONS --------------------

/**
 * Callback for position the servo, using degrees.
 * 
 * @param cmd_msg Integer standart message to write in the servo.  
  */
void servo_cb( const std_msgs::Int32& cmd_msg)
{
  //set servo angle, should be from 0° to 180°  
  servo.write(cmd_msg.data);

  //Toggle led  
  digitalWrite(13, HIGH-digitalRead(13));  
}

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
  return;
}

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

void setup() {
  // Initialize communication
  nh.initNode();
  nh.advertise(pub_range);
  
  // Define a message draft to update 
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;

  // Attach servo to the given pin
  servo.attach(3); 
  
  // Configure the given ports
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  
  // Generate subscriber for motor direction
  ros::Subscriber<std_msgs::Int32> sub2("linear_move", &lineal_callback);
  nh.subscribe(sub2);
}

void loop()
{
  // Publish new message for the ultrasonic's reading
  if ( millis() >= range_time ){
      int r =0;  
      range_msg.range = getRange() / 100;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
      range_time =  millis() + 50;
    }    
    nh.spinOnce();
}
