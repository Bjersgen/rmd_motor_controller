#ifndef rmd_motor_controller_h
#define rmd_motor_controller_h

#include <iostream>
#include <ros/ros.h> 
#include <std_msgs/Empty.h>  
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/Int16.h>
#include <std_msgs/String.h> 
#include <std_msgs/Float32.h> 
#include <std_msgs/Int16MultiArray.h>
using namespace std;


uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength);
void RS_powerControl(uint8_t Motor_ID, int16_t powerControl);
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl);
void RS_Motor_Off(uint8_t Motor_ID);
void RS_Motor_On(uint8_t Motor_ID);
void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1);
void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed);
void RS_readSingleAngle(uint8_t Motor_ID);


void serialRec_cb(const ros::TimerEvent &event);

void cmdRec_cb_frontleft(std_msgs::Empty::ConstPtr &msg);
void cmdRec_cb_frontright(std_msgs::Empty::ConstPtr &msg);
void cmdRec_cb_rearleft(std_msgs::Empty::ConstPtr &msg);
void cmdRec_cb_rearright(std_msgs::Empty::ConstPtr &msg);

void serialSend(uint8_t *TxData, uint8_t len);
void decodeRecData(uint8_t *data);
void singlePositionControl_cb();

#endif
