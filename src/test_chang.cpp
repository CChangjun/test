#include "ros/ros.h"
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

#include "time.h"

#define CONTROL_MOTOR_SPEED_FREQUENCY 30 //hz
#define CONTROL_MOTOR_TIMEOUT 500 //ms
#define IMU_PUBLISH_FREQUENCY 200 //hz
#define CMD_VEL_PUBLISH_FREQUENCY 30 //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY 10 //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY 1 //hz 
#define DEBUG_LOG_FREQUENCY 10 //hz 

#define LINEAR 0
#define ANGULAR 1
#define WHEEL_NUM 2

#define WHEEL_RADIUS 0.0625 //휠 반지름 m단위 

#define TURNING_RADIUS 0.080 

#define MAX_LINEAR_VELOCITY 300//(WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY 50//(MAX_LINEAR_VELOCITY / TURNING_RADIUS) // rad/s

#define MIN_LINEAR_VELOCITY -300 //-MAX_LINEAR_VELOCITY 
#define MIN_ANGULAR_VELOCITY -50 //-MAX_ANGULAR_VELOCITY 

#define LEFT 0
#define RIGHT 1

#define PI 3.14159265359

float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

unsigned char cmd_flag = 0;

ros::Time rosNow(void);
clock_t t; 
clock_t end;
static uint32_t tTime[10];

/*************************Function*************************************/
float constrain(float input, float low, float high);
void updateGoalVelocity(void);

nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;

sensor_msgs::JointState joint_states;

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
tTime[6] = clock();
cmd_flag = 1;

}

int main(int argc, char* argv[])
{
ros::init(argc, argv, "core_serial");
ros::NodeHandle nh;
geometry_msgs::Twist twist;

ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_serial", 100);
ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, commandVelocityCallback);
while(ros::ok()){
ros::spinOnce();
t = clock();

if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
{
updateGoalVelocity();

if(cmd_flag == 1)
{
twist.linear.x = goal_velocity[LINEAR];
twist.angular.z = goal_velocity[ANGULAR];
cmd_vel_pub.publish(twist); 
cmd_flag = 0;
}

tTime[0] = t;
}

}
return 0;
}

void updateGoalVelocity(void)
{
goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

float constrain(float input, float low, float high)
{
if (input < low)
{ input = low;}
else if(input > high)
{ input = high;}
else
{ input = input;}

return input;
}
