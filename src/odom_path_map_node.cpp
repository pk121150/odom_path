#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include "eigen_conversions/eigen_msg.h"
#include <Eigen/Dense>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <iostream>
using Eigen::Matrix4d;
using Eigen::Vector4d;

using namespace std;

visualization_msgs::Marker line_strip_odom;
visualization_msgs::Marker line_strip_odom_combined;
visualization_msgs::Marker line_strip_vio;
visualization_msgs::Marker line_strip_localization;

ros::Publisher marker_pub_odom;
ros::Publisher marker_pub_odom_combined;
ros::Publisher marker_pub_vio;
ros::Publisher marker_pub_localization;

geometry_msgs::Point odom_now;
geometry_msgs::Point odom_combined_now;
geometry_msgs::Point vio_now;
geometry_msgs::Point localization_now;

int count1 = 0;
int count2 = 0;
int count3 = 0;

bool get_first_odom = false;
bool get_first_odom_combined = false;
bool get_first_vio = false;

nav_msgs::Odometry odom_first;
nav_msgs::Odometry odom_combined_first;
nav_msgs::Odometry vio_first;

tf::StampedTransform transform_init;
tf::StampedTransform transform_localization;

tf::Transform transform_odom;
tf::Transform transform_odom_map;
tf::Transform transform_odom_combined;
tf::Transform transform_odom_combined_map;
tf::Transform transform_vio;
tf::Transform transform_vio_map;

void odomCallback1(const nav_msgs::Odometry msg)
{
  if(!get_first_odom)
  {
    get_first_odom = true;
    odom_first = msg;
    transform_odom.setOrigin( tf::Vector3(odom_first.pose.pose.position.x, odom_first.pose.pose.position.y, odom_first.pose.pose.position.z));
    transform_odom.setRotation(tf::Quaternion(odom_first.pose.pose.orientation.x,odom_first.pose.pose.orientation.y,odom_first.pose.pose.orientation.z,odom_first.pose.pose.orientation.w));
    transform_odom_map = transform_init * transform_odom.inverse();
  }else{
    count1++;

    if(count1%10 == 0)
    {

      Vector4d pointE,pointE_map;

      pointE[0] = msg.pose.pose.position.x;
      pointE[1] = msg.pose.pose.position.y;
      pointE[2] = msg.pose.pose.position.z;
      pointE[3] = 1;

      Eigen::Affine3d ta = Eigen::Affine3d::Identity();
      Eigen::Matrix4d tm = Eigen::Matrix4d::Identity();

      tf::transformTFToEigen(transform_odom_map,ta);
      tm = ta.matrix();

      pointE_map = tm*pointE;

      odom_now.x =  pointE_map[0];
      odom_now.y =  pointE_map[1];
      odom_now.z =  pointE_map[2];
      line_strip_odom.points.push_back(odom_now);
      marker_pub_odom.publish(line_strip_odom);

    }
  }


  static tf::TransformBroadcaster br;
  tf::Transform transform_;
  transform_.setOrigin( tf::Vector3(odom_now.x, odom_now.y, odom_now.z));
  transform_.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "/map", "/encoder"));

}

void odomCallback2(const nav_msgs::Odometry msg)
{
  if(!get_first_odom_combined)
  {
    get_first_odom_combined = true;
    odom_combined_first = msg;
    transform_odom_combined.setOrigin( tf::Vector3(odom_combined_first.pose.pose.position.x, odom_combined_first.pose.pose.position.y, odom_combined_first.pose.pose.position.z));
    transform_odom_combined.setRotation(tf::Quaternion(odom_combined_first.pose.pose.orientation.x,odom_combined_first.pose.pose.orientation.y,odom_combined_first.pose.pose.orientation.z,odom_combined_first.pose.pose.orientation.w));
    transform_odom_combined_map = transform_init * transform_odom_combined.inverse();
  }else{
    count2++;

    if(count2%6 == 0)
    {

      Vector4d pointE,pointE_map;

      pointE[0] = msg.pose.pose.position.x;
      pointE[1] = msg.pose.pose.position.y;
      pointE[2] = msg.pose.pose.position.z;
      pointE[3] = 1;

      Eigen::Affine3d ta = Eigen::Affine3d::Identity();
      Eigen::Matrix4d tm = Eigen::Matrix4d::Identity();

      tf::transformTFToEigen(transform_odom_combined_map,ta);
      tm = ta.matrix();

      pointE_map = tm*pointE;

      odom_combined_now.x =  pointE_map[0];
      odom_combined_now.y =  pointE_map[1];
      odom_combined_now.z =  pointE_map[2];
      line_strip_odom_combined.points.push_back(odom_combined_now);
      marker_pub_odom_combined.publish(line_strip_odom_combined);

    }
  }


  static tf::TransformBroadcaster br;
  tf::Transform transform_;
  transform_.setOrigin( tf::Vector3(odom_combined_now.x, odom_combined_now.y, odom_combined_now.z));
  transform_.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "/map", "/ekf_odom"));

}

void odomCallback3(const nav_msgs::Odometry msg)
{
  if(!get_first_vio)
  {
    get_first_vio = true;
    vio_first = msg;
    transform_vio.setOrigin( tf::Vector3(vio_first.pose.pose.position.x,vio_first.pose.pose.position.y, vio_first.pose.pose.position.z));
    transform_vio.setRotation(tf::Quaternion(vio_first.pose.pose.orientation.x,vio_first.pose.pose.orientation.y,vio_first.pose.pose.orientation.z,vio_first.pose.pose.orientation.w));
    transform_vio_map = transform_init * transform_vio.inverse();
  }else{
    count3++;

    if(count3%40 == 0)
    {

      Vector4d pointE,pointE_map;

      pointE[0] = msg.pose.pose.position.x;
      pointE[1] = msg.pose.pose.position.y;
      pointE[2] = msg.pose.pose.position.z;
      pointE[3] = 1;

      Eigen::Affine3d ta = Eigen::Affine3d::Identity();
      Eigen::Matrix4d tm = Eigen::Matrix4d::Identity();

      tf::transformTFToEigen(transform_vio_map,ta);
      tm = ta.matrix();

      pointE_map = tm*pointE;

      vio_now.x =  pointE_map[0];
      vio_now.y =  pointE_map[1];
      vio_now.z =  pointE_map[2];
      line_strip_vio.points.push_back(vio_now);
      marker_pub_vio.publish(line_strip_vio);

    }
  }


  static tf::TransformBroadcaster br;
  tf::Transform transform_;
  transform_.setOrigin( tf::Vector3(vio_now.x, vio_now.y, vio_now.z));
  transform_.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "/map", "/vio"));

}








int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_path_map_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback1);
  ros::Subscriber sub1 = n.subscribe("/odom_combined", 1000, odomCallback2);
  ros::Subscriber sub2 = n.subscribe("/t265/odom/sample", 1000, odomCallback3);

  marker_pub_odom = n.advertise<visualization_msgs::Marker>("/odom_marker",1);
  marker_pub_odom_combined = n.advertise<visualization_msgs::Marker>("/odom_combined_marker",1);
  marker_pub_vio = n.advertise<visualization_msgs::Marker>("/vio_marker",1);
  marker_pub_localization = n.advertise<visualization_msgs::Marker>("/localization_marker",1);

  line_strip_odom.header.frame_id = "/map";
  line_strip_odom.ns = "linestrip";
  line_strip_odom.action = visualization_msgs::Marker::ADD;
  line_strip_odom.pose.orientation.w = 1.0;
  line_strip_odom.id = 1;
  line_strip_odom.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_odom.scale.x = 0.2;
  line_strip_odom.color.r = 1.0;
  line_strip_odom.color.a = 1.0;

  line_strip_odom_combined.header.frame_id = "/map";
  line_strip_odom_combined.ns = "linestrip";
  line_strip_odom_combined.action = visualization_msgs::Marker::ADD;
  line_strip_odom_combined.pose.orientation.w = 1.0;
  line_strip_odom_combined.id = 2;
  line_strip_odom_combined.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_odom_combined.scale.x = 0.2;
  line_strip_odom_combined.color.g = 1.0;
  line_strip_odom_combined.color.a = 1.0;

  line_strip_vio.header.frame_id = "/map";
  line_strip_vio.ns = "linestrip";
  line_strip_vio.action = visualization_msgs::Marker::ADD;
  line_strip_vio.pose.orientation.w = 1.0;
  line_strip_vio.id = 3;
  line_strip_vio.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_vio.scale.x = 0.2;
  line_strip_vio.color.b = 1.0;
  line_strip_vio.color.a = 1.0;

  line_strip_localization.header.frame_id = "/map";
  line_strip_localization.ns = "linestrip";
  line_strip_localization.action = visualization_msgs::Marker::ADD;
  line_strip_localization.pose.orientation.w = 1.0;
  line_strip_localization.id = 4;
  line_strip_localization.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_localization.scale.x = 0.2;
  line_strip_localization.color.r = 1.0;
  line_strip_localization.color.b = 1.0;
  line_strip_localization.color.a = 1.0;


  tf::TransformListener listener;



  int a = 1;
  while (a == 1){

    try{
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform_init);
      a = 0;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      a = 1;
      continue;
    }

  }


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    try{
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform_localization);

      localization_now.x = transform_localization.getOrigin().x();
      localization_now.y = transform_localization.getOrigin().y();
      localization_now.z = transform_localization.getOrigin().z();

      line_strip_localization.points.push_back(localization_now);
      marker_pub_localization.publish(line_strip_localization);
    }
    catch (tf::TransformException &ex) {
      continue;
    }


    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}
