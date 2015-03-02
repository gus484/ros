#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_transform_core.h"

using std::string;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "laser_transform");
  ros::NodeHandle n;
 
  // Declare variables thar can be modified by launch file or command line.
  int rate;
  string sub_topic;
  string pub_topic;
 
  // Create a new LaserTransformer object.
  LaserTransform *node_lt = new LaserTransform();

  node_lt->init();

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("sub_topic", sub_topic, string("/cloud"));
  private_node_handle_.param("pub_topic", pub_topic, string("/cloud_world"));

  // Create a subscriber
  ros::Subscriber sub_message = n.subscribe(sub_topic.c_str(), 1000, &LaserTransform::messageCallback, node_lt);

  // Create a publisher and name the topic
  ros::Publisher pub_message = n.advertise<sensor_msgs::PointCloud2>(pub_topic.c_str(), 50);

  ros::Rate r(rate);

  while (n.ok())
  {
    node_lt->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
  // end main
}
