#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
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
  bool mf_msgs;
  bool gps_msgs;
  bool odo_msgs;
  string pcl_in_topic;
  string pcl_out_topic;
  string mf_topic;
  string imu_topic;
  string gps_topic;
  string odo_topic;
  string odo_topic_filtered;

  // Create a new LaserTransformer object.
  LaserTransform *node_lt = new LaserTransform();

  node_lt->init();

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("mf_msgs", mf_msgs, bool(false));
  private_node_handle_.param("imu_msgs", imu_msgs, bool(true));
  private_node_handle_.param("gps_msgs", gps_msgs, bool(false));
  private_node_handle_.param("odo_msgs", odo_msgs, bool(true));
  private_node_handle_.param("pcl_in_topic", pcl_in_topic, string("/cloud"));
  private_node_handle_.param("pcl_out_topic", pcl_out_topic, string("/cloud_world"));
  private_node_handle_.param("mf_topic", mf_topic, string("magnetic/data"));
  private_node_handle_.param("imu_topic", imu_topic, string("/imu/data"));
  private_node_handle_.param("gps_topic", gps_topic, string("/gps/fix"));
  private_node_handle_.param("odo_topic", odo_topic, string("odom"));
  private_node_handle_.param("odo_filtered", odo_topic_filtered, string("/odometry/filtered"));
  private_node_handle_.param("imu_convergence_speed", imu_convergence_speed, int(20));

  // Create a subscriber for laser scanner plc data
  ros::Subscriber sub_pcl = n.subscribe(pcl_in_topic.c_str(), 1000, &LaserTransform::callbackPcl, node_lt);

  // Create a subscriber for filtered odometry data
  ros::Subscriber sub_odo = n.subscribe(odo_topic_filtered.c_str(), 1000, &LaserTransform::callbackOdometryFiltered, node_lt);

  // Create a publisher for transformed plc msgs
  ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>(pcl_out_topic.c_str(), 50);
  node_lt->setPclPublisher(&pcl_pub);
  
  // Create a publisher for transformed plc msgs
  //ros::Publisher pub_joint = n.advertise<sensor_msgs::JointState Message>("fix_multicar", 50);

  // Create a publisher for magnetic field msgs
  ros::Publisher mf_pub = n.advertise<sensor_msgs::MagneticField>(mf_topic.c_str(), 50);

  // Create a publisher for IMU msgs
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic.c_str(), 50);

  // Create a publisher for GPS msgs
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>(gps_topic.c_str(), 50);
  
  // Create a publisher for odometry mesgs
  ros::Publisher odo_pub = n.advertise<nav_msgs::Odometry>(odo_topic.c_str(),50);

  ros::Rate r(rate);

  while (n.ok())
  {
    //node_lt->publishPclMessage(&pcl_pub);
    if (imu_msgs)
      node_lt->publishImuMessage(&imu_pub);
    if (gps_msgs)
      node_lt->publishNavSatFixMessage(&gps_pub);
    if (mf_msgs)
      node_lt->publishMagneticFieldMessage(&mf_pub);
    if (odo_msgs)
      node_lt->publishOdometryMessage(&odo_pub);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
  // end main
}
