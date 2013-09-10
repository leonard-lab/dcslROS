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
//#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <dcsl_messages/belugaInput.h> //For adding custom messages see http://www.ros.org/wiki/rosserial_arduino/Tutorials/Adding%20Custom%20Messages
#include <std_srvs/Empty.h>

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

//Depth calibration values
int air;
int bottom;

//Declare NodeHandle and Int16 message objects
ros::NodeHandle nh;
using std_srvs::Empty;
std_msgs::Float32 cal_depth;
std_msgs::Int16 raw_depth;

//Declare Publisher object
ros::Publisher depth("depth_measurement", &cal_depth);
ros::Publisher rdepth("raw_depth", &raw_depth);

unsigned long last_cmd; //time the last command was sent. for timeout of motors
int current_reading; //global for the current depth reading

//Callback function for receiving motor inputs
void command( const dcsl_messages::belugaInput& input){
  int vm;
  int tm;
  float sv;
  
  // Constrain commands to realizable values
  if (abs(input.vertical_motor) > 255){
    vm = 255 * signum(input.vertical_motor);
  }
  else{
    vm = input.vertical_motor;
  }
  if (abs(input.thrust_motor) > 255){
    tm = 255 * signum(input.thrust_motor);
  }
  else{
    tm = input.thrust_motor;
  }
  if (abs(input.servo) > 3.1415926/2.0){
    sv = 3.1415926/2.0 * float(signum(input.servo));
  }
  else{
    sv = input.servo;
  }
  
  // Do commands
  if (vm > 0){
    digitalWrite(dir2Pin, LOW);
  }
  else {
    digitalWrite(dir2Pin, HIGH);
  }
  analogWrite(en2Pin, abs(vm));
  
  if(tm > 0){
    digitalWrite(dir1Pin, HIGH);
  }
  else{
    digitalWrite(dir1Pin, LOW);
  }  
  analogWrite(en1Pin, abs(tm));
  
  int servo_deg = int(sv * 180.0/3.1415926);
  int pos = servo_deg + 90;
  servo.write(pos);
  
  last_cmd = millis();
}

//Declare Subscriber object
ros::Subscriber<dcsl_messages::belugaInput> sub("cmd_inputs", &command);

//Declare RunningAverage object
const int numReadings = 10; //Number of analog readings for the running average
RunningAverage RA(numReadings);

//Callback function for air calibration service
void air_callback(const Empty::Request& req, Empty::Response& res)
{
  air = current_reading;
}
//Callback function for bottom calibration service
void bottom_callback(const Empty::Request& req, Empty::Response& res)
{
  bottom = current_reading;
}

//Declare ROS service objects
ros::ServiceServer<Empty::Request, Empty::Response> air_server("set_air_calibration", &air_callback);
ros::ServiceServer<Empty::Request, Empty::Response> bottom_server("set_bottom_calibration", &bottom_callback);

void setup()
{
  delay(250);
  nh.initNode(); //Initialized the node
  delay(250);
  nh.advertise(depth); //Advertise the topic
  nh.advertise(rdepth);
  nh.advertiseService(air_server);
  nh.advertiseService(bottom_server);
  delay(250);
  nh.subscribe(sub);
  delay(250);
  
  nh.getParam("~air_reading", &air);
  nh.getParam("~bottom_reading", &bottom);
  
  RA.clear(); //Clear the running average
  
  servo.attach(servoPin);
  
  pinMode(en1Pin, OUTPUT); digitalWrite(en1Pin, LOW);
  pinMode(en2Pin, OUTPUT); digitalWrite(en2Pin, LOW);
  pinMode(dir1Pin, OUTPUT); digitalWrite(dir1Pin, LOW);
  pinMode(dir2Pin, OUTPUT); digitalWrite(dir2Pin, LOW);
  
  pinMode(2,OUTPUT); digitalWrite(2,HIGH);
  
  //Start motors off
  analogWrite(en1Pin, 0);
  analogWrite(en2Pin, 0);
}

float z_reading_air = 2.3876; //meters
float z_reading_water = z_reading_air - 2.0066; //meters

void loop() {
  RA.addValue(analogRead(sensorPin)); //Add depth sensor reading to the running average
  cal_depth.data = (z_reading_air - z_reading_water)/(float(air) - float(bottom))*(float(RA.getAverage()) - float(air)) + z_reading_air; //Output current average reading to message
  current_reading = RA.getAverage();
  raw_depth.data = RA.getAverage();
  depth.publish( &cal_depth); //Publish current running average depth reading  
  rdepth.publish( &raw_depth);
  //ROS_DEBUG_STREAM_THROTTLE_NAMED(5, "beluga", "Thruster Current = " << analogRead(current1Pin) << "; Vertical Current = " << analogRead(current2Pin));
  
  //Stop motors if no commands have been received for 1 second
  if (millis() - last_cmd > 1000){
    analogWrite(en1Pin, 0);
    analogWrite(en2Pin, 0);
  }
  
  nh.spinOnce();
}

//Signum function
static inline int8_t signum(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

