#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>

visualization_msgs::Marker line_strip_odom;
visualization_msgs::Marker line_strip_odom_combined;
visualization_msgs::Marker line_strip_vio;

ros::Publisher marker_pub_odom;
ros::Publisher marker_pub_odom_combined;
ros::Publisher marker_pub_vio;

geometry_msgs::Point odom_now;
geometry_msgs::Point odom_combined_now;
geometry_msgs::Point vio_now;

int count1 = 0;
int count2 = 0;
int count3 = 0;

void odomCallback1(const nav_msgs::Odometry msg)
{
  count1++;

  if(count1%10 == 0)
  {
    odom_now.x =  msg.pose.pose.position.x;
    odom_now.y =  msg.pose.pose.position.y;
    odom_now.z =  msg.pose.pose.position.z;
    line_strip_odom.points.push_back(odom_now);
    marker_pub_odom.publish(line_strip_odom);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_now.x, odom_now.y, odom_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_path_frame", "/encoder"));

}

void odomCallback2(const nav_msgs::Odometry msg)
{
  count2++;

  if(count2%6 == 0)
  {
    odom_combined_now.x =  msg.pose.pose.position.x;
    odom_combined_now.y =  msg.pose.pose.position.y;
    odom_combined_now.z =  msg.pose.pose.position.z;
    line_strip_odom_combined.points.push_back(odom_combined_now);
    marker_pub_odom_combined.publish(line_strip_odom_combined);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(odom_combined_now.x, odom_combined_now.y, odom_combined_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_path_frame", "/encoder_imu"));
}

void odomCallback3(const nav_msgs::Odometry msg)
{
  count3++;

  if(count3%40 == 0)
  {
    vio_now.x =  msg.pose.pose.position.x;
    vio_now.y =  msg.pose.pose.position.y;
    vio_now.z =  msg.pose.pose.position.z;
    line_strip_vio.points.push_back(vio_now);
    marker_pub_vio.publish(line_strip_vio);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(vio_now.x, vio_now.y, vio_now.z));
  transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom_path_frame", "/vio"));
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_path_node");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback1);
  ros::Subscriber sub1 = n.subscribe("/odom_combined", 1000, odomCallback2);
  ros::Subscriber sub2 = n.subscribe("/t265/odom/sample", 1000, odomCallback3);

  marker_pub_odom = n.advertise<visualization_msgs::Marker>("/odom_marker",1);
  marker_pub_odom_combined = n.advertise<visualization_msgs::Marker>("/odom_combined_marker",1);
  marker_pub_vio = n.advertise<visualization_msgs::Marker>("/vio_marker",1);

  line_strip_odom.header.frame_id = "/odom_path_frame";
  line_strip_odom.ns = "linestrip";
  line_strip_odom.action = visualization_msgs::Marker::ADD;
  line_strip_odom.pose.orientation.w = 1.0;
  line_strip_odom.id = 1;
  line_strip_odom.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_odom.scale.x = 0.2;
  line_strip_odom.color.r = 1.0;
  line_strip_odom.color.a = 1.0;

  line_strip_odom_combined.header.frame_id = "/odom_path_frame";
  line_strip_odom_combined.ns = "linestrip";
  line_strip_odom_combined.action = visualization_msgs::Marker::ADD;
  line_strip_odom_combined.pose.orientation.w = 1.0;
  line_strip_odom_combined.id = 2;
  line_strip_odom_combined.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_odom_combined.scale.x = 0.2;
  line_strip_odom_combined.color.g = 1.0;
  line_strip_odom_combined.color.a = 1.0;

  line_strip_vio.header.frame_id = "/odom_path_frame";
  line_strip_vio.ns = "linestrip";
  line_strip_vio.action = visualization_msgs::Marker::ADD;
  line_strip_vio.pose.orientation.w = 1.0;
  line_strip_vio.id = 3;
  line_strip_vio.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_vio.scale.x = 0.2;
  line_strip_vio.color.b = 1.0;
  line_strip_vio.color.a = 1.0;


  ros::spin();

  return 0;
}
