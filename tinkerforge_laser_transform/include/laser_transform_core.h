#ifndef LASER_TRANSFORM_CORE_H
#define LASER_TRANSFORM_CORE_H

// ROS includes
#include <fstream>
#include "ros/ros.h"
#include "ros/time.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "ip_connection.h"
#include "brick_imu.h"
#include "bricklet_gps.h"
#include "bricklet_industrial_dual_0_20ma.h"

#define M_PI	3.14159265358979323846  /* pi */

class LaserTransform
{
public:
  //! Constructur
  LaserTransform();
  
  //! Destructor
  ~LaserTransform();

  //! Init
  int init();

  //! Publish the transformed plc data.
  void publishPclMessage(ros::Publisher *pub_message);

  //! Publish the IMU message.
  void publishImuMessage(ros::Publisher *pub_message);

  //! Publish the NavSatFix message.
  void publishNavSatFixMessage(ros::Publisher *pub_message);

  //! Callcack function for laser scanner pcl.
  void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  //! Callback function for odometry
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
public:
  //! IMU convergence_speed setter
  int setImuConvergenceSpeed(uint16_t imu_convergence_speed)
  {
    this->imu_convergence_speed = imu_convergence_speed;
    return 0;
  }
private:
  //! Callback function for Tinkerforge ip connected .
  static void connectedCallback(uint8_t connect_reason, void *user_data);
 
  //! Callback function for Tinkerforge enumerate.
  static void enumerateCallback(const char *uid, const char *connected_uid,
    char position, uint8_t hardware_version[3],
    uint8_t firmware_version[3], uint16_t device_identifier,
    uint8_t enumeration_type, void *user_data);

  //! Callback function for Tinkerforge Industrial Dual 0-20mA Bricklet
   static void dual020Callback(uint8_t sensor, int32_t current, void *user_data);

  //! Get IMU quaternion.
  tf::Quaternion getQuaternion();

  //! Get current car position
  int getPosition(float *x_pos, float *y_pos, float *z_pos);

  //! Get current car velocity
  int getVelocity(float *velocity);
  //! Calculate deg from rad
  float rad2deg(float x)
  {
    return x*180.0/M_PI;
  }
private:
  //! IP connection to Tinkerforge deamon.
  IPConnection ipcon;
  //! The IMU device.
  IMU imu;
  //! The IMU state
  bool is_imu_connected;
  //! The IMU convergence_speed
  int imu_convergence_speed;
  //! The GPS device.
  GPS gps;
  //! The GPS state.
  bool is_gps_connected;
  //! The IndustrialDual020mA device.
  IndustrialDual020mA dual020;
  //! The IndustrialDual020mA state.
  bool is_dual020_connected;
  //! Counter for IndustrialDual020mA triggers.
  uint32_t dual020_trigger_cnt_c1;
  uint32_t dual020_trigger_cnt_c2;
  //! yaw angle.
  float yaw;
  //! pitch angle.
  float pitch;
  //! roll angle.
  float roll;
  //! The transformed point cloud
  sensor_msgs::PointCloud2 pcl_out;
  bool publish_new_pcl;
  //! start latitude
  double start_latitude;
  //! start longitude
  double start_longitude;
  //! laser scanner orientation
  tf::Quaternion laser_orientation;
  //! The current car position on x-axis
  int xpos;
  //! The current car position on y-axis
  int ypos;
  //! The current vehicle velocity
  float velocity;
  //! GPS-Logfile
  std::fstream gps_log;
};

#endif
