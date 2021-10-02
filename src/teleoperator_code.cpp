#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

geometry_msgs::Twist desired_velocity;
sensor_msgs::LaserScan lidar_reading;
geometry_msgs::Twist command_velocity;

void velCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  desired_velocity.angular = vel_msg->angular;
  desired_velocity.linear = vel_msg->linear;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg)
{
  lidar_reading.ranges = lidar_msg->ranges;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleoperator");

  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber vel_sub = n.subscribe("des_vel", 1000, velCallback);
  ros::Subscriber lidar_sub = n.subscribe("laser_1", 1000, lidarCallback);

  ros::Rate loop_rate(10); 

  while (ros::ok())
  {
    if (lidar_reading.ranges.size() != 0) 
    {
      ROS_INFO("Lidar is: [%f]", lidar_reading.ranges.at(134));
    }
    else  
    {
      ROS_DEBUG_THROTTLE(30, "There is no robot on the map");
    }
    

    command_velocity = desired_velocity;
    if (lidar_reading.ranges.size() != 0 &&
        lidar_reading.ranges.at(134) < 1.0 &&
        command_velocity.linear.x > 0.0)
    {
      command_velocity.linear.x = 0.0;
    }

    vel_pub.publish(command_velocity);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}