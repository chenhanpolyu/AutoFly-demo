#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "stdio.h"
#include <fstream>

using namespace  std;

ros::Publisher repub;
ros::Publisher velrepub;
ros::Subscriber sub;
bool enable_output_file;
bool enable_repub;
std::ofstream fd;
std::string output_file_path;
std::string sub_topic;
std::string sub_type;
std::string repub_topic;
std::string repub_type;
bool if_pubvel;
nav_msgs::Path path;
int frame_id_path = 0;

static void republishtopic(ros::Time stamp, double x, double y, double z,
                           double qw, double qx, double qy, double qz)
{
  if(repub_type == "PoseStamped")
  {
    geometry_msgs::PoseStamped latest_pose;
    latest_pose.header.frame_id = "map";
    latest_pose.header.stamp=stamp;
    latest_pose.pose.orientation.w = qw;
    latest_pose.pose.orientation.x = qx;
    latest_pose.pose.orientation.y = qy;
    latest_pose.pose.orientation.z = qz;
    latest_pose.pose.position.x = x;
    latest_pose.pose.position.y = y;
    latest_pose.pose.position.z = z;
    repub.publish(latest_pose);
  }
  if(repub_type == "Path")
  {
    geometry_msgs::PoseStamped latest_pose;
    latest_pose.header.frame_id = "map";
    latest_pose.header.stamp=stamp;
    latest_pose.pose.orientation.w = qw;
    latest_pose.pose.orientation.x = qx;
    latest_pose.pose.orientation.y = qy;
    latest_pose.pose.orientation.z = qz;
    latest_pose.pose.position.x = x;
    latest_pose.pose.position.y = y;
    latest_pose.pose.position.z = z;

    path.header.frame_id = "map";
    path.header.stamp = stamp;
    path.poses.push_back(latest_pose);
//    if(path.poses.size()>=1000)
//    {
//      path.poses.erase(path.poses.begin());
//    }
    repub.publish(path);
  }
}


static void process(ros::Time stamp, double x, double y, double z,
               double qw, double qx, double qy, double qz)
{
  static ros::Time last_time=ros::Time::now();
  if((ros::Time::now().toSec()-last_time.toSec())>0.1)
  {
    if(enable_output_file)
    {
      fd << setprecision(6)
         << stamp << " "
         << setprecision(9)
         << x << " "
         << y << " "
         << z << " "
         << qw << " "
         << qx << " "
         << qy << " "
         << qz << std::endl;
    }
    if(enable_repub)
    {
      republishtopic(stamp,x,y,z,qw,qx,qy,qz);
    }
  }
}

void TransformStamped_callback(const geometry_msgs::TransformStampedConstPtr msg)
{
  process(msg->header.stamp,
          msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z,
          msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y,  msg->transform.rotation.z);
}

void PointStamped_callback(const geometry_msgs::PointStampedConstPtr msg)
{
  process(msg->header.stamp,
          msg->point.x, msg->point.y, msg->point.z,
          1, 0, 0, 0);
}

void PoseStamped_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
  process(msg->header.stamp,
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
          msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void Odometry_callback(const nav_msgs::OdometryConstPtr msg)
{
//  cout<<"call_back, go to process"<<endl;
  process(msg->header.stamp,
          msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
          msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (if_pubvel)
{
    geometry_msgs::TwistStamped vel;
    vel.header.frame_id = msg->header.frame_id;
    vel.header.stamp= msg->header.stamp;
    vel.twist.linear = msg->twist.twist.linear;
    vel.twist.angular = msg->twist.twist.angular;
  //  vel.twist.angular.y = qx;
  //  vel.twist.angular.z = qy;
  //  vel.twist.linear.x = qz;
  //  vel.twist.linear.y = y;
  //  vel.twist.linear.z = z;
    velrepub.publish(vel);
}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "repub_rec");
  ros::NodeHandle nh("~");

  nh.getParam("sub_topic",        sub_topic);
  nh.getParam("sub_type",         sub_type);
  nh.getParam("repub_topic",      repub_topic);
  nh.getParam("repub_type",       repub_type);
  nh.getParam("if_pubvel",       if_pubvel);
  nh.getParam("output_file_path", output_file_path);
//  cout << sub_topic << endl;
  if(output_file_path=="0")
  {
    enable_output_file = false;
  }else
  {
    enable_output_file = true;
    fd.open(output_file_path.c_str());
  }
  if(repub_type == "0")
  {
    enable_repub = false;
  }
  else
  {
    enable_repub = true;
    if(repub_type == "PoseStamped")
    {
      repub = nh.advertise<geometry_msgs::PoseStamped>(repub_topic, 2);
      if (if_pubvel)
       {
      velrepub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 2);}
    }
    if(repub_type == "Path")
    {
       repub = nh.advertise<nav_msgs::Path>(repub_topic, 2);;
    }
  }

  if(sub_type=="TransformStamped")
  {
    sub = nh.subscribe(sub_topic, 2, TransformStamped_callback);
  }
  if(sub_type=="PointStamped")
  {
    sub = nh.subscribe(sub_topic, 2, PointStamped_callback);
  }
  if(sub_type=="PoseStamped")
  {
    sub = nh.subscribe(sub_topic, 2, PoseStamped_callback);
  }
  if(sub_type=="Odometry")
  {
    sub = nh.subscribe(sub_topic, 2, Odometry_callback);
  }

  ros::spin();

  return 0;
}
