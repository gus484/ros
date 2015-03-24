#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "laser_transform_core.h"

using std::string;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "laser_transform");
  ros::NodeHandle n;

  // Declare variables thar can be modified by launch file or command line.
  int rate;
  int imu_convergence_speed;
  bool imu_msgs;
  bool gps_msgs;
  string pcl_in_topic;
  string pcl_out_topic;
  string imu_topic;
  string gps_topic;
  //string odo_topic;

  // Create a new LaserTransformer object.
  LaserTransform *node_lt = new LaserTransform();

  node_lt->init();

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("imu_msgs", imu_msgs, bool(true));
  private_node_handle_.param("gps_msgs", gps_msgs, bool(false));
  private_node_handle_.param("pcl_in_topic", pcl_in_topic, string("/cloud"));
  private_node_handle_.param("pcl_out_topic", pcl_out_topic, string("/cloud_world"));
  private_node_handle_.param("imu_topic", imu_topic, string("/imu/data"));
  private_node_handle_.param("gps_topic", gps_topic, string("/gps/fix"));
  private_node_handle_.param("imu_convergence_speed", imu_convergence_speed, int(20));

  // Create a subscriber
  ros::Subscriber sub_message = n.subscribe(pcl_in_topic.c_str(), 1000, &LaserTransform::pclCallback, node_lt);

  // Create odometry subscriber
  ros::Subscriber sub_odometry = n.subscribe("/odometry/filtered", 50, &LaserTransform::odometryCallback, node_lt);

  // Create a publisher and name the topic
  ros::Publisher pub_message = n.advertise<sensor_msgs::PointCloud2>(pcl_out_topic.c_str(), 50);

  // Create a publisher for IMU msgs
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic.c_str(), 50);

  // Create a publisher for GPS msgs
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>(gps_topic.c_str(), 50);

  ros::Rate r(rate);

  while (n.ok())
  {
    node_lt->publishPclMessage(&pub_message);
    if (imu_msgs)
      node_lt->publishImuMessage(&imu_pub);
    if (gps_msgs)
      node_lt->publishNavSatFixMessage(&gps_pub);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
  // end main
}
