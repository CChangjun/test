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
#include <math.h>
#include "dong_core/sensor.h" 
#include "time.h"

#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define LINEAR                           0
#define ANGULAR                          1
#define WHEEL_NUM                        2

#define WHEEL_RADIUS                     0.125 //휠 반지름 m단위 모바일로봇 기준 이것만 바꿈   
#define WHEEL_SEPARATION                 0.160
#define TURNING_RADIUS                   0.080 

#define MAX_LINEAR_VELOCITY              200//(WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             50//(MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY  
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY 

#define LEFT                             0
#define RIGHT                            1
#define TURNING_RADIUS                   0.080 
#define TICK2RAD                         0.001533981

float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
unsigned long prev_update_time;

char odom_header_frame_id[30];
char odom_child_frame_id[30];
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};
bool init_encoder = true;
double last_diff_tick[WHEEL_NUM] = {0, 0};
float yaw_angle = 0;
float odom_pose[3];
double odom_vel[3];
#define FIRMWARE_VER "1.2.6"
unsigned char flag = 0;
unsigned char cmd_flag = 0;

bool isChecked;
ros::Time rosNow(void);
clock_t t; 
clock_t end;
static uint32_t tTime[10];
float constrain(float input, float low, float high);
void updateGoalVelocity(void);
void updateVariable(bool isConnected);


nav_msgs::Odometry odom;

geometry_msgs::TransformStamped odom_tf;
sensor_msgs::JointState joint_states;
//////////////////////////////////
tf::TransformBroadcaster *br;
tf::Transform transform;

void broadcast_dummy_to_base_link_transform() {
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "footprint", "base_link"));
}///////////////////////////

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = clock();
  cmd_flag = 1;

}

void msgCallback(const dong_core::sensor::ConstPtr& msg)
{
  
  unsigned long time_now = clock();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = ros::Time::now();

  flag = 1;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "chang_core");
  ros::NodeHandle nh;
  

  geometry_msgs::Twist twist;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_serial", 100);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, commandVelocityCallback);

  br = new tf::TransformBroadcaster();////////

  char get_prefix[10];
  std::string get_tf_prefix = get_prefix;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  while(ros::ok()){
  ros::spinOnce();
  t = clock();
  broadcast_dummy_to_base_link_transform(); //////
  isChecked = false;
    
 

    if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY)) //33ms 마다 출력 되는거 확인
    {
      updateGoalVelocity();
      if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT) 
      {
        if(cmd_flag == 1)
        {
          cmd_vel_pub.publish(twist);
          cmd_flag = 0;
        } 
      } 
      else {
        if(cmd_flag == 1)
        {
          twist.linear.x = goal_velocity[LINEAR];
          twist.angular.z = goal_velocity[ANGULAR];
          ROS_INFO("Data sent to ATmega128: linear=%.2f, angular=%.2f",twist.linear.x,twist.angular.z);
          ROS_INFO("%.2f,%.2f",twist.linear.x ,twist.angular.z);
          ROS_INFO("-*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_*_-");
          cmd_vel_pub.publish(twist); 
          cmd_flag = 0;
        }
      }
      tTime[0] = t;
    }

    if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
    {
      //sensor_msg.quest = true;
     
   
      if(flag == 1)
      {
        ros::Time stamp_now = ros::Time::now();
       
      }
      tTime[2] = t;
    }

    if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
    {
      tTime[3] = t;
    }


  }
  delete br;////
  return 0;
}

void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

float constrain(float input, float low, float high)
{
  if (input < low)
  {  input = low;}
  else if(input > high)
  {  input = high;}
  else
  { input = input;}

  return input;
}


