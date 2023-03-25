#include <iostream>
#include <math.h>
#include"ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/String.h> 
#include <std_msgs/Float32.h> 

using namespace std;

// Define the four wheels of the robot
int frontLeft = 0;
int frontRight = 1;
int rearLeft = 2;
int rearRight = 3;

// Define the wheel angles
double wheelAngle = 45.0;
double sinVal = 21.4;
double cosVal = 15.5;

// Define the motor speed for each wheel
int motorSpeed[4] = {0, 0, 0, 0};

// Function to calculate the motor speed for each wheel
void calculateMotorSpeed(double x, double y, double rot) {
  // Calculate the motor speed for the front left wheel
  motorSpeed[frontLeft] = round(x - y - (rot * (cosVal + sinVal)));

  // Calculate the motor speed for the front right wheel
  motorSpeed[frontRight] = round(x + y + (rot * (cosVal + sinVal)));

  // Calculate the motor speed for the rear left wheel
  motorSpeed[rearLeft] = round(x + y - (rot * (cosVal + sinVal)));

  // Calculate the motor speed for the rear right wheel
  motorSpeed[rearRight] = round(x - y + (rot * (cosVal + sinVal)));
}

// Main function to control the robot movement
int main(int argc, char** argv) {

  ros::init(argc,argv,"motion_controller_node");

  ros::NodeHandle nh;

  ros::Publisher pub1 = nh.advertise<std_msgs::Int16>("/rmd/cmd/frontleft",4);
  ros::Publisher pub2 = nh.advertise<std_msgs::Int16>("/rmd/cmd/frontright",4);
  ros::Publisher pub3 = nh.advertise<std_msgs::Int16>("/rmd/cmd/rearleft",4);
  ros::Publisher pub4 = nh.advertise<std_msgs::Int16>("/rmd/cmd/rearright",4);

  ros::Rate loop_rate(20);
  double x = 0.0;  // X-axis movement
  double y = 0.0;  // Y-axis movement
  double rot = 0.0;  // Rotation

  // Example movement commands
  x = 0.5; // Move forward
  y = 0.0; // No sideways movement
  rot = 0.2; // Rotate to the right

  // Calculate the motor speed for each wheel based on the movement commands
  calculateMotorSpeed(x, y, rot);
  while(ros::ok()){
     std_msgs::Int16 msg1,msg2,msg3,msg4;
     msg1.data = motorSpeed[frontLeft];
     msg2.data = motorSpeed[frontRight];
     msg3.data = motorSpeed[rearLeft];
     msg4.data = motorSpeed[rearRight];
     pub1.publish(msg1);
     pub2.publish(msg2);
     pub3.publish(msg3);
     pub4.publish(msg4);
     loop_rate.sleep();
     cout << "Front left: " << motorSpeed[frontLeft] << endl;
     cout << "Front right: " << motorSpeed[frontRight] << endl;
     cout << "Rear left: " << motorSpeed[rearLeft] << endl;
     cout << "Rear right: " << motorSpeed[rearRight] << endl;

  }
  // Print the motor speed for each wheel

  return 0;
}
