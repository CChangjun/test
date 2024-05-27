#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <map>
#include <serial/serial.h> // 추가
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstring> // for memcpy

#define PI 3.141592
#define FUNCTION_WRITE 6
#define FUNCTION_ADDRESS 120
#define RPM_ADDRESS 121
#define FUNCTION_DATA_START 1
#define FUNCTION_DATA_STOP 2
#define RPM_MAX 2000
#define WHEEL_DIAMETER 0.125 //휠 직경 12.5cm
#define WHEELBASE 0.28
#define MOTOR_REDUCER 20.0
#define LEFT_ID 2
#define RIGHT_ID 1

//MY_COMMAND_BELOW
#define LINEAR_X_MAX 100
#define LINEAR_X_MIN -100

#define ANGULAR_Z_MAX 30
#define ANGULAR_Z_MIN -30

// serial port settings
serial::Serial ser;
// read 
using namespace std;

/**************************Variable*************************************/

int what;
float speed(0.5); // Linear velocity (m/s)
float turn(0.5); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');
unsigned char BufferIndex = 0;
char BufferPacket[50] = {0,};
/**********************************************************************/


/**************************Function*************************************/
void ser_msgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  uint8_t linear_x_8bit =  (uint8_t)(msg->linear.x);
  uint8_t angular_z_8bit = (uint8_t)(msg->angular.z);

  snprintf(BufferPacket, sizeof(BufferPacket), "%.1f,%.1f\n", msg->linear.x, msg->angular.z);
  //### initializing argument :Serial::write(const uint8_t*, size_t)’
  ser.write(BufferPacket);
  //ROS_INFO("send fin %d,%d", BufferPacket[0],BufferPacket[1]);
  ROS_INFO("--------------------------------");
}


void msgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
//데이터가 토픽에 도달하였는지 확인하기 위해 사용
  ROS_INFO("Linear.x %3.f", msg->linear.x);   
  ROS_INFO("Angular.z %3.f", msg->angular.z);   
                                          
  uint8_t linear_x_8bit = static_cast<uint8_t>(msg->linear.x);
  uint8_t angular_z_8bit = static_cast<uint8_t>(msg->angular.z);
  
  if(ser.isOpen())
  {
      ROS_INFO("Data sent to ATmega128: linear_x_8bit=%u, angular_z_8bit=%u", linear_x_8bit, angular_z_8bit);
      ROS_INFO("--------------------------------------------");
  } 
  else
  {
      ROS_ERROR("Serial port not open");
  }

  if(key == 115) //ASCII code for s
  {
    speed = 0;
    turn = 0;
  }

}

int main(int argc, char **argv)                         
{
  ros::init(argc, argv, "serial_core"); 
  ros::NodeHandle nh;
  ros::Subscriber serial_sub = nh.subscribe("cmd_vel_serial", 1, ser_msgCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1, msgCallback);
  ros::Publisher read_pub = nh.advertise<std_msgs::String>("read",1);         

  try
  {

      ser.setPort("/dev/ttyFT232");
      //ser.setPort("/dev/ttyATmegaHTNC");
      //ser.setPort("/dev/ttySTM32");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(2000);
      ser.setTimeout(to);
      ser.open();
  }

  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }else{
     return -1;
  }

  
  ros::Rate loop_rate(500);

  while(ros::ok()){
    
    ros::spinOnce();
    if(ser.available())
    {
        std_msgs::String read_msg;
        ROS_INFO_STREAM("Reading from ATmega128");
        read_msg.data = ser.read(ser.available());
        ROS_INFO_STREAM("READ: " << read_msg.data);
        read_pub.publish(read_msg);
    }

    loop_rate.sleep();

  }
}

