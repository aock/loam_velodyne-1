#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>


ros::Publisher pubOdom;
ros::Publisher pubInt;
std::string velodyne_frame;
std::string robot_frame;
std::string odom_frame;

void odomCallback(const nav_msgs::Odometry odom)
{
    ROS_INFO("odom received");
    nav_msgs::Odometry odom_out = odom;
    odom_out.header.frame_id = odom_frame;
    odom_out.child_frame_id = robot_frame;

    double x, y, z;
    
    pubOdom.publish(odom_out);
}

void intCallback(const nav_msgs::Odometry odom)
{
    nav_msgs::Odometry odom_out = odom;
    odom_out.header.frame_id = odom_frame;
    odom_out.child_frame_id = robot_frame;
    ROS_INFO("integrated received");

    geometry_msgs::Pose pose;

    pose = odom.pose.pose;

    odom_out.pose.pose.position.x = pose.position.z;
    odom_out.pose.pose.position.y = pose.position.y;
    odom_out.pose.pose.position.z = pose.position.x;

    pubInt.publish(odom_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformOdometry");
  ros::NodeHandle nh;

  ROS_INFO("Started Transform Odometry");

  nh.param<std::string>("velodyne_frame", velodyne_frame, "velodyne");
  nh.param<std::string>("robot_frame", robot_frame, "base_footprint");
  nh.param<std::string>("odom_frame", odom_frame, "odom_combined");

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> 
                                  ("/laser_odom_to_init", 2, odomCallback);

  ros::Subscriber subInt = nh.subscribe<nav_msgs::Odometry> 
                                  ("/integrated_to_init", 2, intCallback);

  pubOdom = nh.advertise<nav_msgs::Odometry> ("/laser_odom", 5);

  pubInt = nh.advertise<nav_msgs::Odometry> ("/laser_integrated", 5);

  ros::spin();

  return 0;
}