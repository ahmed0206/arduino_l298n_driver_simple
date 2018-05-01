/*  Arduino DC Motor Control - PWM | H-Bridge | L298N
    Using ROS msgs to actuate 5V DC motors with Arduino UNO
    by Vinh Nguyen 05/01/2018
*/
//including relevant libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>

//including ros_lib
//#include <ArduinoHardware.h>
//#include <ArduinoTcpHardware.h>

//defining ports used on Arduino UNO
#define enA 9 //activate motor
//#define in1 4 //activate port 4
//#define in2 5 //activate port 5
//#define enB 10 //activate motor
#define in3 6 //active port 6
#define in4 7 //activate port 7

float input; //declaring global input value 
int motorSpeedA = 0; //defining PWM
int motorSpeedB = 0; //defining PWM

//This is a formatting shortcut to redefine ros::NodeHandle > nh, so that the code further down looks neat and pretty. You can actually name this to anything as long as you keep the same naming convention
ros::NodeHandle nh;

geometry_msgs::Twist value;

// This is the callback function that executes the final command. In this case the arduino will actuate the 5V DC motors
//Because we are only subscribing to one input, we define it as const

void motor_actuation_cb(const geometry_msgs::Twist &msg)
{
  
  input=msg.linear.x;
  // Backward
  if (input > 0) {
    // Move Motor
    //digitalWrite(in1, HIGH);
    //digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining input readings for going backward from -.5 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(input, 0, .8, 0, 255);
    //motorSpeedB = 255;
  } 
    else {
    motorSpeedA = 70;
    //motorSpeedB = 70; 
    }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  //analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}

//This sets up the ROS Subscriber node so the arduino can have the framework to communicate on ROS
//ros::Subscriber object topic name("name of topic to subscribe to", &callback function);
ros::Subscriber<geometry_msgs::Twist> motor_driver("key_vel", &motor_actuation_cb);

void setup()
{
  nh.initNode();
  nh.subscribe(motor_driver);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}

