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
//visualization_msgs::Marker line_strip_odom_combined_xsens;

ros::Publisher marker_pub_odom;
ros::Publisher marker_pub_odom_combined;
ros::Publisher marker_pub_vio;
//ros::Publisher marker_pub_odom_combined_xsens;

geometry_msgs::Point start_point_map;

geometry_msgs::Point odom_now;
geometry_msgs::Point odom_combined_now;
geometry_msgs::Point vio_now;
//geometry_msgs::Point odom_combined_xsens_now;

int count1 = 0;
int count2 = 0;
int count3 = 0;
//int count4 = 0;

bool get_first_odom = false;
bool get_first_odom_combined = false;
bool get_first_vio = false;

nav_msgs::Odometry odom_first;
nav_msgs::Odometry odom_combined_first;
nav_msgs::Odometry vio_first;


tf::StampedTransform transform_init;

tf::Transform transform_odom;
tf::Transform transform_odom_combined;
tf::Transform transform_vio;

void odomCallback1(const nav_msgs::Odometry msg)
{
  if(!get_first_odom)
  {
    get_first_odom = true;
    odom_first = msg;
  }else{
    count1++;

    if(count1%10 == 0)
    {

      transform_odom.setOrigin( tf::Vector3(odom_first.pose.pose.position.x, odom_first.pose.pose.position.y, odom_first.pose.pose.position.z));
      transform_odom.setRotation(tf::Quaternion(odom_first.pose.pose.orientation.x,odom_first.pose.pose.orientation.y,odom_first.pose.pose.orientation.z,odom_first.pose.pose.orientation.w));


      Vector4d pointE,pointE_odom,pointE_map;

      pointE[0] = msg.pose.pose.position.x;
      pointE[1] = msg.pose.pose.position.y;
      pointE[2] = msg.pose.pose.position.z;
      pointE[3] = 1;

      Eigen::Affine3d ta = Eigen::Affine3d::Identity();
      Eigen::Matrix4d tm = Eigen::Matrix4d::Identity();

      Eigen::Affine3d ta_odom = Eigen::Affine3d::Identity();
      Eigen::Matrix4d tm_odom = Eigen::Matrix4d::Identity();

      tf::transformTFToEigen(transform_init,ta);
      tm = ta.matrix();

      tf::transformTFToEigen(transform_odom.inverse(),ta_odom);
      tm_odom = ta_odom.matrix();
      //cout<<tm<<endl;
      /*tm(0,3) = start_point_map.x;
      tm(1,3) = start_point_map.y;
      tm(2,3) = start_point_map.z;
      */pointE_odom = tm_odom*pointE;
      pointE_map = tm*pointE_odom;
      odom_now.x =  pointE_map[0];
      odom_now.y =  pointE_map[1];
      odom_now.z =  pointE_map[2];
      line_strip_odom.points.push_back(odom_now);
      marker_pub_odom.publish(line_strip_odom);


      //tf::transformEigenToTF(transform_base_to_scan_d, transform_tf);
      /*odom_now.x =  msg.pose.pose.position.x + start_point_map.x;
      odom_now.y =  msg.pose.pose.position.y + start_point_map.y;
      odom_now.z =  msg.pose.pose.position.z + start_point_map.z;
      line_strip_odom.points.push_back(odom_now);
      marker_pub_odom.publish(line_strip_odom);*/
    }
  }


  /*static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_now.x, odom_now.y, odom_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/encoder"));*/

}

void odomCallback2(const nav_msgs::Odometry msg)
{
  count2++;

  if(count2%6 == 0)
  {
    odom_combined_now.x =  msg.pose.pose.position.x + start_point_map.x;
    odom_combined_now.y =  msg.pose.pose.position.y + start_point_map.y;
    odom_combined_now.z =  msg.pose.pose.position.z + start_point_map.z;
    line_strip_odom_combined.points.push_back(odom_combined_now);
    marker_pub_odom_combined.publish(line_strip_odom_combined);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_combined_now.x, odom_combined_now.y, odom_combined_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/encoder_imu"));
}

void odomCallback3(const nav_msgs::Odometry msg)
{
  count3++;

  if(count3%40 == 0)
  {
    vio_now.x =  msg.pose.pose.position.x + start_point_map.x;;
    vio_now.y =  msg.pose.pose.position.y + start_point_map.y;
    vio_now.z =  msg.pose.pose.position.z + start_point_map.z;
    line_strip_vio.points.push_back(vio_now);
    marker_pub_vio.publish(line_strip_vio);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(vio_now.x, vio_now.y, vio_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/vio"));
}

/*void odomCallback4(const nav_msgs::Odometry msg)
{
  count4++;

  if(count4%6 == 0)
  {
    odom_combined_xsens_now.x =  msg.pose.pose.position.x + start_point_map.x;;
    odom_combined_xsens_now.y =  msg.pose.pose.position.y + start_point_map.y;;
    odom_combined_xsens_now.z =  msg.pose.pose.position.z + start_point_map.z;;
    line_strip_odom_combined_xsens.points.push_back(odom_combined_xsens_now);
    marker_pub_odom_combined_xsens.publish(line_strip_odom_combined_xsens);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_combined_xsens_now.x, odom_combined_xsens_now.y, odom_combined_xsens_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/encoder_imu_xsens"));
}*/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_path_map_node");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback1);
  ros::Subscriber sub1 = n.subscribe("/odom_combined", 1000, odomCallback2);
  ros::Subscriber sub2 = n.subscribe("/t265/odom/sample", 1000, odomCallback3);
  //ros::Subscriber sub3 = n.subscribe("/odom_combined_xsens", 1000, odomCallback4);

  marker_pub_odom = n.advertise<visualization_msgs::Marker>("/odom_marker",1);
  marker_pub_odom_combined = n.advertise<visualization_msgs::Marker>("/odom_combined_marker",1);
  marker_pub_vio = n.advertise<visualization_msgs::Marker>("/vio_marker",1);
  //marker_pub_odom_combined_xsens = n.advertise<visualization_msgs::Marker>("/odom_combined_xsens_marker",1);

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

/*  line_strip_odom_combined_xsens.header.frame_id = "/map";
  line_strip_odom_combined_xsens.ns = "linestrip";
  line_strip_odom_combined_xsens.action = visualization_msgs::Marker::ADD;
  line_strip_odom_combined_xsens.pose.orientation.w = 1.0;
  line_strip_odom_combined_xsens.id = 4;
  line_strip_odom_combined_xsens.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_odom_combined_xsens.scale.x = 0.2;
  line_strip_odom_combined_xsens.color.r = 1.0;
  line_strip_odom_combined_xsens.color.b = 1.0;
  line_strip_odom_combined_xsens.color.a = 1.0;*/

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


  start_point_map.x = transform_init.getOrigin().x();
  start_point_map.y = transform_init.getOrigin().y();
  start_point_map.z = transform_init.getOrigin().z();




  ros::spin();

  return 0;
}
