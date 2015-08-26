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
#include "brick_imu_v2.h"
#include "bricklet_gps.h"
#include "bricklet_industrial_digital_in_4.h"
#include "bricklet_dual_button.h"
#include "octomap_msgs/BoundingBoxQuery.h"

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

  //! Publish the transformed plc data
  void publishPclMessage(ros::Publisher *pub_message);

  //! Publish the IMU message
  void publishImuMessage(ros::Publisher *pub_message);

  //! Publish the NavSatFix message.
  void publishNavSatFixMessage(ros::Publisher *pub_message);

  //! Publish the MagneticField message
  void publishMagneticFieldMessage(ros::Publisher *pub_message);

  //! Publish the current car velocity
  void publishOdometryMessage(ros::Publisher *pub_message);

  //! Callcack function for laser scanner pcl
  void callbackPcl(const sensor_msgs::PointCloud2::ConstPtr& msg);

  //! Callcack function for filtered odometry
  void callbackOdometryFiltered(const nav_msgs::Odometry::ConstPtr& msg);

  //! Set the laser pose
  void setLaserPose(double x, double y, double z, double yaw, double pitch, double roll);

  //! Set the speed of convergence depends on the angular velocity.
  void checkConvergenceSpeed();

  //! Set the unnecessary part of the octomap free
  void clearOctomap(ros::ServiceClient *client);

  //! Set publisher for transformed pcl data
  void setPclPublisher(ros::Publisher *pub)
  {
    pcl_pub = pub;
  }
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

  //! Callback function for Tinkerforge Industrial Digital In 4 Bricklet
  static void idi4Callback(uint8_t interrupt_mask, uint8_t value_mask, void *user_data);

  //! Callback function for Tinkerforge Dual Button Bricklet
  static void dbCallback(uint8_t button_l, uint8_t button_r,
                      uint8_t led_l, uint8_t led_r,
                      void *user_data);

  //! Get IMU quaternion.
  tf::Quaternion getQuaternion();

  //! Get current car position
  int getPosition(float *x_pos, float *y_pos, float *z_pos);

  //! Calculate deg from rad
  float rad2deg(float x)
  {
    return x*180.0/M_PI;
  }

  //! Calculate rad from deg
  float deg2rad(float x)
  {
    return x*M_PI/180.0;
  }
private:
  //! Publisher for transfered pcl data
  ros::Publisher *pcl_pub;
  //! IP connection to Tinkerforge deamon.
  IPConnection ipcon;
  //! The IMU device.
  IMU imu;
  //! The IMU state
  bool is_imu_connected;
  //! The IMU convergence_speed
  int imu_convergence_speed;
  //! The IMU_v2 device.
  IMUV2 imu_v2;
  //! The IMU_v2 state
  bool is_imu_v2_connected;
  //! The IMU_v2 convergence_speed
  int imu_v2_convergence_speed;
  //! The GPS device.
  GPS gps;
  //! The GPS state.
  bool is_gps_connected;
  //! The IndustrialDual020mA device.
  IndustrialDigitalIn4 idi4;
  //! Counter for rounds per second (na)
  uint16_t rpm_cnt;
  //! The IndustrialDual020mA state.
  bool is_idi4_connected;
  //! The DualButton device.
  DualButton db;
  //! Flag for active velocity measurement
  bool isMeasure;
  //! The rev from the inductive proximity switch sensor
  float rev;
  //! Time since the last rev
  ros::Time last_rev;
  //! The transformed point cloud
  sensor_msgs::PointCloud2 pcl_out;
  //! Flag for new transformed pcl available
  bool publish_new_pcl;
  //! Flag for new raw pcl available
  bool new_pcl_filtered;
  //! start latitude
  double start_latitude;
  //! start longitude
  double start_longitude;
  //! laser scanner pose
  tf::Transform laser_pose;
  //! laser scanner orientation
  //tf::Quaternion laser_orientation;
  //! 
  bool isPlc;
  //! The current car position on x-axis
  float xpos;
  //! The current car position on y-axis
  float ypos;
  //! The current vehicle velocity
  float velocity;
  //! The current vehicle velocity
  float velocity_gps;
  //! The GPS course
  float course_gps;
  float yy;
  //! GPS-Logfile
  std::fstream gps_log;
  //! Velocity-Logfile
  std::fstream velo_log;
  //! Full-Logfile
  std::fstream full_log;
};

#endif
