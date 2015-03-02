#ifndef LASER_TRANSFORM_CORE_H
#define LASER_TRANSFORM_CORE_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "ip_connection.h"
#include "brick_imu.h"
#include "bricklet_gps.h"

class LaserTransform
{
public:
  //! Constructur
  LaserTransform();
  
  //! Destructor
  ~LaserTransform();

  //! Init
  int init();

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callcack function for subscriber.
  void messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
private:
  //! Callback function for Tinkerforge ip connected .
  static void connectedCallback(uint8_t connect_reason, void *user_data);
 
  //! Callback function for Tinkerforge enumerate.
  static void enumerateCallback(const char *uid, const char *connected_uid,
    char position, uint8_t hardware_version[3],
    uint8_t firmware_version[3], uint16_t device_identifier,
    uint8_t enumeration_type, void *user_data);

  //! Get IMU quaternion.
  tf::Quaternion getQuaternion();
  
  //! Get position
  int getPosition(float *x_pos, float *y_pos, float *z_pos);
private:
  //! IP connection to Tinkerforge deamon.
  IPConnection ipcon;
  //! The IMU device.
  IMU imu;
  //! The IMU state
  bool is_imu_connected;
  //! The GPS device.
  GPS gps;
  //! The GPS state.
  bool is_gps_connected;
  //! yaw angle.
  float yaw;
  //! pitch angle.
  float pitch;
  //! roll angle.
  float roll;
  sensor_msgs::PointCloud2 pcl_out;
  bool publish_new_pcl;
};

#endif
