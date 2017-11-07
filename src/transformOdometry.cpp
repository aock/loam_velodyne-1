#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
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
double velodyne_x, velodyne_y, velodyne_z;
double odom_corr_x, odom_corr_y, odom_corr_z;
double odom_corr_roll, odom_corr_pitch, odom_corr_yaw;

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
    odom_out.header.frame_id = "camera_init";
    odom_out.child_frame_id = "camera";
    // ROS_INFO("integrated received");

    geometry_msgs::Quaternion orientation = odom.pose.pose.orientation;
    tf::Quaternion or_tf, corr_tf;
    
    tf::quaternionMsgToTF(orientation, or_tf);

    corr_tf.setRPY(odom_corr_roll, odom_corr_pitch, odom_corr_yaw);

    tf::Quaternion res_quat = or_tf * corr_tf;
    
    tf::quaternionTFToMsg(res_quat, orientation);

    odom_out.pose.pose.orientation = orientation;

    pubInt.publish(odom_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformOdometry");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  ROS_INFO("Started Transform Odometry");

  nh.param<std::string>("velodyne_frame", velodyne_frame, "velodyne");
  nh.param<std::string>("robot_frame", robot_frame, "base_footprint");
  nh.param<std::string>("odom_frame", odom_frame, "odom_combined");
  nh.param<std::string>("camera_init", camera_init, "camera_init");

  // velodyne params
  nh.param<double>("velodyne_roll", velodyne_roll, 0.1);
  nh.param<double>("velodyne_pitch", velodyne_pitch, 0.25);
  nh.param<double>("velodyne_yaw", velodyne_yaw, 0.0);
  nh.param<double>("velodyne_x", velodyne_x, 0.264);
  nh.param<double>("velodyne_y", velodyne_y, 0.000);
  nh.param<double>("velodyne_z", velodyne_z, 0.397);

  // odom correction params
  nh.param<double>("odom_corr_x", odom_corr_x, 0);
  nh.param<double>("odom_corr_y", odom_corr_y, 0);
  nh.param<double>("odom_corr_z", odom_corr_z, 0);
  nh.param<double>("odom_corr_roll", odom_corr_roll, 0);
  nh.param<double>("odom_corr_pitch", odom_corr_pitch, 0);
  nh.param<double>("odom_corr_yaw", odom_corr_yaw, -M_PI/2 );
  

  ROS_INFO("Started Transform Odometry");
  ROS_INFO_STREAM("" << velodyne_roll << " " << velodyne_pitch << " " << velodyne_yaw);
  
  m_tf_listener_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener);


  ros::Subscriber subOdom = n.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 2, odomCallback);
  ros::Subscriber subInt = n.subscribe<nav_msgs::Odometry>("/integrated_to_init", 2, intCallback);

  pubOdom = n.advertise<nav_msgs::Odometry>("/laser_odom/raw",10);
  pubInt = n.advertise<nav_msgs::Odometry>("/laser_odom/integrated",10);

  static tf::TransformBroadcaster br;
  
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( velodyne_x, velodyne_y, velodyne_z) );
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