
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#define USE_USBCON

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>
#include <ArduinoHardware.h>

// #define USE_USBCON

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

int pingPin = A5; // trigger 
int inPin = A4; // echo
long range_time;
char frameid[] = "/ultrasound";
int vel_x = 250;

Servo servo;

void servo_cb( const std_msgs::Int32& cmd_msg){
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}



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

void setup() {
  nh.initNode();
  nh.advertise(pub_range);
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.0;
  range_msg.max_range = 6.47;
  nh.initNode();
  // ros::Subscriber<std_msgs::Int32> sub("servo", servo_cb);
  // nh.subscribe(sub);
  servo.attach(3); //attach it to pin 9
  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
 
  ros::Subscriber<std_msgs::Int32> sub2("linear_move", &lineal_callback);
  nh.subscribe(sub2);
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

long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29.1 / 2;
}

float getRange()
{
    
    // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);
  
  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}
