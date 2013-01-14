/*
 Beluga Microcontroller Software
 Created by Brendan Andrade on December 4, 2012
 */

/*
 This code uses the RunningAverages library available at
 http://playground.arduino.cc//Main/RunningAverage
 */
#include <RunningAverage.h>

#include <Servo.h>

#include <ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <dcsl_messages/belugaInput.h> //For adding custom messages see http://www.ros.org/wiki/rosserial_arduino/Tutorials/Adding%20Custom%20Messages


int sensorPin = 5; //Analog pin to which the depth sensor is attached
int servoPin = 9; //PWM pin for to which servo is attached

//Motor 1 is the horizontal thruster
//Motor 2 in the vertical thruster

int en1Pin = 3; //Enable (PWM) pin for motor 1
int en2Pin = 11; //Enable (PWM) pin for motor 2
int dir1Pin = 12; //Direction pin for motor 1
int dir2Pin = 13; //Direction pin for motor 2
//int current1Pin = 1; //Current sense for motor 1
//int current2Pin = 0; //Current sense for motor 2

//Create servo object
Servo servo;

//Declare NodeHandle and Int16 message objects
ros::NodeHandle nh;
std_msgs::Int16 rawDepth;

//Declare Publisher object
ros::Publisher depth("depth_measurement", &rawDepth);

//Callback function for receiving motor inputs
void command( const dcsl_messages::belugaInput& input){
  if (input.vertical_motor > 0){
    digitalWrite(dir2Pin, HIGH);
  }
  else {
    digitalWrite(dir2Pin, LOW);
  }
  analogWrite(en2Pin, abs(input.vertical_motor));
  
  if(input.thrust_motor > 0){
    digitalWrite(dir1Pin, HIGH);
  }
  else{
    digitalWrite(dir1Pin, LOW);
  }  
  analogWrite(en1Pin, abs(input.thrust_motor));
  
  int pos = input.servo + 90;
  servo.write(pos);
}

//Declare Subscriber object
ros::Subscriber<dcsl_messages::belugaInput> sub("cmd_inputs", &command);

//Declare RunningAverage object
const int numReadings = 10; //Number of analog readings for the running average
RunningAverage RA(numReadings);

void setup()
{
  nh.initNode(); //Initialized the node
  nh.advertise(depth); //Advertise the topic
  nh.subscribe(sub);
  
  RA.clear(); //Clear the running average
  
  servo.attach(servoPin);
  
  pinMode(en1Pin, OUTPUT); digitalWrite(en1Pin, LOW);
  pinMode(en2Pin, OUTPUT); digitalWrite(en2Pin, LOW);
  pinMode(dir1Pin, OUTPUT); digitalWrite(dir1Pin, LOW);
  pinMode(dir2Pin, OUTPUT); digitalWrite(dir2Pin, LOW);
  
  //Start motors off
  analogWrite(en1Pin, 0);
  analogWrite(en2Pin, 0);
}

void loop() {
  RA.addValue(analogRead(sensorPin)); //Add depth sensor reading to the running average
  rawDepth.data = RA.getAverage(); //Output current average reading to message
  depth.publish( &rawDepth); //Publish current running average depth reading  
  
  //ROS_DEBUG_STREAM_THROTTLE_NAMED(5, "beluga", "Thruster Current = " << analogRead(current1Pin) << "; Vertical Current = " << analogRead(current2Pin));
  
  nh.spinOnce();
}
