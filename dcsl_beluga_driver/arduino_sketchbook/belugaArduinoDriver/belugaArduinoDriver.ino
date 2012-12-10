/*
 Beluga Microcontroller Software
 Created by Brendan Andrade on December 4, 2012
 */

/*
 This code uses the RunningAverages library available at
 http://playground.arduino.cc//Main/RunningAverage
 */
#include "RunningAverage.h"

/*
 This code uses the AFMotor library available at
 http://www.ladyada.net/make/mshield/download.html
 */  
#include "AFMotor.h"

#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <dcsl_messages/belugaInput.h> //For adding custom messages see http://www.ros.org/wiki/rosserial_arduino/Tutorials/Adding%20Custom%20Messages


int sensorPin = 1; //Analog pin to which the depth sensor is attached
int servoPin = 9; //PWM pin for to which servo is attached

//Declare motor objects
AF_DCMotor vert(2, MOTOR12_64KHZ);
AF_DCMotor thrust(1, MOTOR12_64KHZ);
Servo servo;

//Declare NodeHandle and Int16 message objects
ros::NodeHandle nh;
std_msgs::Int16 rawDepth;

//Declare Publisher object
ros::Publisher depth("depth_measurement", &rawDepth);

//Callback function for receiving motor inputs
void command( const dcsl_messages::belugaInput& input){
  if (input.vertical_motor > 0){
    vert.run(FORWARD);
  }
  else if(input.vertical_motor < 0){
    vert.run(BACKWARD);
  }
  else {
    vert.run(RELEASE);
  }
  vert.setSpeed(abs(input.vertical_motor));
  
  if(input.thrust_motor > 0){
    thrust.run(FORWARD);
  }
  else if(input.thrust_motor < 0){
    thrust.run(BACKWARD);
  }
  else{
    thrust.run(RELEASE);
  }  
  thrust.setSpeed(abs(input.thrust_motor));
  
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
  RA.clr(); //Clear the running average
  servo.attach(servoPin);
}

void loop() {
  RA.add(analogRead(sensorPin)); //Add depth sensor reading to the running average
  rawDepth.data = RA.avg(); //Output current average reading to message
  depth.publish( &rawDepth); //Publish current running average depth reading  
  nh.spinOnce();
}
