#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <tf/transform_listener.h>

boost::shared_ptr<tf::TransformListener> m_tf_listener_ptr;

ros::Publisher pubOdom;
ros::Publisher pubInt;
std::string velodyne_frame;
std::string robot_frame;
std::string odom_frame;
std::string camera_init;

double velodyne_roll, velodyne_pitch, velodyne_yaw;

void odomCallback(const nav_msgs::Odometry odom)
{
    // ROS_INFO("odom received");
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
    // ROS_INFO("integrated received");

    geometry_msgs::Pose pose;

    pose = odom.pose.pose;

    // axis correction of position

    tf::Quaternion rot;
    rot.setRPY(velodyne_roll, velodyne_pitch, velodyne_yaw);

    tf::Transform trans(rot);

    tf::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    
    
    position = trans * position;


    odom_out.pose.pose.position.x = position[0];
    odom_out.pose.pose.position.y = position[1];
    odom_out.pose.pose.position.z = position[2];

    pubInt.publish(odom_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformOdometry");
  ros::NodeHandle nh("~");

  ROS_INFO("Started Transform Odometry");

  nh.param<std::string>("velodyne_frame", velodyne_frame, "velodyne");
  nh.param<std::string>("robot_frame", robot_frame, "base_footprint");
  nh.param<std::string>("odom_frame", odom_frame, "odom_combined");
  nh.param<std::string>("camera_init", camera_init, "camera_init");
  nh.param<double>("velodyne_roll", velodyne_roll, 0.1);
  nh.param<double>("velodyne_pitch", velodyne_pitch, 0.25);
  nh.param<double>("velodyne_yaw", velodyne_yaw, 0.0);

  ROS_INFO("Started Transform Odometry");
  ROS_INFO_STREAM("" << velodyne_roll << " " << velodyne_pitch << " " << velodyne_yaw);
  
  m_tf_listener_ptr =boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);

  static tf::TransformBroadcaster br;
  
  
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.264, 0.000, 0.397) );
  tf::Quaternion q;
  q.setRPY(velodyne_roll, velodyne_pitch, velodyne_yaw);
  transform.setRotation(q);
  
  while(ros::ok())
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, camera_init));
    ros::spinOnce();
  }

  return 0;
}