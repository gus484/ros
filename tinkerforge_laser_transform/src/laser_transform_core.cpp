#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "laser_transform_core.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>
#include "ip_connection.h"
#include "brick_imu.h"
#include "bricklet_gps.h"
#include "bricklet_industrial_digital_in_4.h"
#include "bricklet_dual_button.h"
#include <tf/transform_broadcaster.h>

#define HOST "localhost"
#define PORT 4223

#define GPS_LOGFILE false
#define VELO_LOGFILE false
#define FULL_SENSOR_LOGFILE true
#define LOGFILE_PATH "/home/vmuser/logs/"

/*----------------------------------------------------------------------
 * LaserTransform()
 * Constructor
 *--------------------------------------------------------------------*/

LaserTransform::LaserTransform()
{
  publish_new_pcl = false;
  is_imu_connected = false;
  is_gps_connected = false;
  is_idi4_connected = false;
  isMeasure = false;
  new_pcl_filtered = false;
  start_latitude = 0;
  start_longitude = 0;
  velocity = 0.0;
  velocity_gps = 0.0;
  imu_convergence_speed = 0;
  rev = 0.0;
  last_rev = ros::Time::now();

  xpos = ypos = 0;

  std::stringstream ss;

  time_t t;
  struct tm *ts;
  char buff[80];

  // build filename
  t = time(NULL);
  ts = localtime(&t);

  // open gps log file
  if (GPS_LOGFILE)
  {
    strftime(buff, 80, "gps_log_%Y_%m_%d-%H_%M_%S.txt", ts);
    ss << LOGFILE_PATH << buff;

    gps_log.open(ss.str().c_str(), std::ios::out);
  }

  // open velocity log file
  if (VELO_LOGFILE)
  {
    strftime(buff, 80, "velo_log_%Y_%m_%d-%H_%M_%S.txt", ts);
    ss << LOGFILE_PATH << buff;

    velo_log.open(ss.str().c_str(), std::ios::out);
  }
}

/*----------------------------------------------------------------------
 * ~LaserTransform()
 * Destructor
 *--------------------------------------------------------------------*/

LaserTransform::~LaserTransform()
{
  if (is_imu_connected)
  {
    // disconnect tinkerforge
    imu_leds_off(&imu);
    ipcon_destroy(&ipcon);
  }
  // close gps logfile
  if (gps_log.is_open())
    gps_log.close();
  // close velo logfile
  if (velo_log.is_open())
    velo_log.close();
}

/*----------------------------------------------------------------------
 * Init()
 * Init the TF-Devices
 *--------------------------------------------------------------------*/

int LaserTransform::init()
{
  // create IP connection
  ipcon_create(&ipcon);

  // connect to brickd
  if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
    std::cout << "could not connect to brickd!" << std::endl;
    return false;
  }

  // register connected callback to "cb_connected"
  ipcon_register_callback(&ipcon,
    IPCON_CALLBACK_CONNECTED,
    (void*)connectedCallback,
    this);

  // register enumeration callback to "cb_enumerate"
  ipcon_register_callback(&ipcon,
    IPCON_CALLBACK_ENUMERATE,
    (void*)enumerateCallback,
    this);

  return 0;
}

/*----------------------------------------------------------------------
 * publishPclMessage()
 * Publish the transformed plc data.
 *--------------------------------------------------------------------*/

void LaserTransform::publishPclMessage(ros::Publisher *pub_message)
{
  if (publish_new_pcl)
  {
    pub_message->publish(pcl_out);
    publish_new_pcl = false;
  }
}

/*----------------------------------------------------------------------
 * publishImuMessage()
 * Publish the Imu message.
 *--------------------------------------------------------------------*/

void LaserTransform::publishImuMessage(ros::Publisher *pub_message)
{
  int16_t acc_x, acc_y, acc_z;
  int16_t mag_x, mag_y, mag_z;
  int16_t ang_x, ang_y, ang_z;
  int16_t temp;
  float x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  static uint32_t seq = 0;
  ros::Time current_time = ros::Time::now();
  tf::TransformBroadcaster tf_broadcaster;
  if (is_imu_connected)
  {
    sensor_msgs::Imu imu_msg;

    imu_get_quaternion(&imu, &x, &y, &z, &w);

    imu_get_all_data(&imu, &acc_x, &acc_y, &acc_z, &mag_x, &mag_y,
      &mag_z, &ang_x, &ang_y, &ang_z, &temp);

    // message header
    imu_msg.header.seq = seq;
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "base_link";

    imu_msg.orientation.x = w;
    imu_msg.orientation.y = z*-1;
    imu_msg.orientation.z = y;
    imu_msg.orientation.w = x*-1;

    // orientation_covariance
    boost::array<const double, 9> oc =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};

    imu_msg.orientation_covariance = oc;
    //imu_msg.orientation_covariance[0] = -1;

    // velocity from °/14.375 to rad/s
    imu_msg.angular_velocity.x = deg2rad(ang_x/14.375);
    imu_msg.angular_velocity.y = deg2rad(ang_y/14.375);
    imu_msg.angular_velocity.z = deg2rad(ang_z/14.375);

    // velocity_covariance
    boost::array<const double, 9> vc =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};
    imu_msg.angular_velocity_covariance = vc;
    //imu_msg.angular_velocity_covariance[0] = -1;

    // acceleration from mG to m/s²
    imu_msg.linear_acceleration.x = (acc_x/1000.0)*9,80605;
    imu_msg.linear_acceleration.y = (acc_y/1000.0)*9,80605;
    imu_msg.linear_acceleration.z = (acc_z/1000.0)*9,80605;

    // linear_acceleration_covariance
    boost::array<const double, 9> lac =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};
    imu_msg.linear_acceleration_covariance = lac;
    //imu_msg.linear_acceleration_covariance[0] = -1;

    pub_message->publish(imu_msg);

    seq++;
  }
}

/*----------------------------------------------------------------------
 * checkConvergenceSpeed()
 * Set the speed of convergence depends on the angular velocity.
 *--------------------------------------------------------------------*/

void LaserTransform::checkConvergenceSpeed()
{
  if (is_imu_connected)
  {
    int16_t ang_x, ang_y, ang_z;
    imu_get_angular_velocity(&imu, &ang_x, &ang_y, &ang_z);
    if (ang_x > 50 || ang_y > 50 || ang_z > 50)
    {
      imu_set_convergence_speed(&imu, 80);
    }
    else
      imu_set_convergence_speed(&imu, imu_convergence_speed);
  }
}

/*----------------------------------------------------------------------
 * publishMagneticFieldMessage()
 * Publish the MagneticField message.
 *--------------------------------------------------------------------*/

void LaserTransform::publishMagneticFieldMessage(ros::Publisher *pub_message)
{
  static uint32_t seq = 0;
  if (is_imu_connected)
  {
    int16_t x = 0, y = 0, z = 0;
    imu_get_magnetic_field(&imu, &x, &y, &z);

    sensor_msgs::MagneticField mf_msg;

    // message header
    mf_msg.header.seq =  seq;
    mf_msg.header.stamp = ros::Time::now();
    mf_msg.header.frame_id = "base_link";

    // magnetic field from mG to T
    mf_msg.magnetic_field.x = x/10000000.0;
    mf_msg.magnetic_field.x = y/10000000.0;
    mf_msg.magnetic_field.x = z/10000000.0;

    for (int i = 0 ; i < 9 ; i++)
      mf_msg.magnetic_field_covariance[i] = 0.01;

    pub_message->publish(mf_msg);

    seq++;
  }
  return;
}

/*----------------------------------------------------------------------
 * publishNavSatFixMessage()
 * Publish the NavSatFix message.
 *--------------------------------------------------------------------*/

void LaserTransform::publishNavSatFixMessage(ros::Publisher *pub_message)
{
  static uint32_t seq = 0;
  uint8_t fix, satellites_view, satellites_used;
  uint16_t pdop, hdop, vdop, epe;
  uint32_t latitude, longitude;
  uint32_t altitude, geoidal_separation;
  uint32_t course, speed;
  char ns, ew;
  if (is_gps_connected)
  {
    // get gps sensor status
    gps_get_status(&gps, &fix, &satellites_view, &satellites_used);

    if (fix != GPS_FIX_3D_FIX)
      return; // No valid data

    gps_get_coordinates(&gps, &latitude, &ns, &longitude, &ew, &pdop,
      &hdop, &vdop, &epe);
    gps_get_altitude(&gps, &altitude, &geoidal_separation);
    // course in deg, speed in 1/100 km/h
    gps_get_motion(&gps, &course, &speed);

    if (this->velocity > 0.0)
    {
      this->velocity_gps = (speed*100) / 3.6; // in m/s²
      this->course_gps = deg2rad(course*100);
      ROS_INFO_STREAM("GPS-Velocity:" << this->velocity_gps);
      ROS_INFO_STREAM("GPS-Course:" << this->course_gps);
    }
    else
    {
      this->velocity_gps = 0.0;
      this->course_gps = 0.0;
    }

    sensor_msgs::NavSatFix gps_msg;

    // message header
    gps_msg.header.seq =  seq;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = "gps";
    // gps status
    gps_msg.status.status = gps_msg.status.STATUS_SBAS_FIX;
    gps_msg.status.service = gps_msg.status.SERVICE_GPS;

    gps_msg.latitude = latitude/1000000.0;
    gps_msg.longitude = longitude/1000000.0;
    gps_msg.altitude = altitude/100.0;
    gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN;

    if (gps_log.is_open() && GPS_LOGFILE)
    {
      std::string separator = "|";
      gps_log << ros::Time::now().sec;
      gps_log << gps_log << separator;
      gps_log << gps_msg.latitude;
      gps_log << separator;
      gps_log << gps_msg.longitude;
      gps_log << separator;
      gps_log << gps_msg.altitude;
      gps_log << std::endl;
    }

    geographic_msgs::GeoPoint ll;
    // create UTM from point
    ll = geodesy::toMsg(gps_msg);
    geodesy::UTMPoint pt;
    geodesy::fromMsg(ll,pt);
    if (xpos == 0 && ypos == 0)
    {
      xpos = pt.easting;
      ypos = pt.northing;
    }
    else
    {
      //ROS_INFO_STREAM("COORDS_1:" << pt.easting << "::" << pt.northing);
      //ROS_INFO_STREAM("COORDS_2:" << xpos << "::" << ypos);

      //std::cout << "COORDS_3:" << (int)(xpos-pt.easting) << "::" << (int)(ypos-pt.northing) << std::endl;
      //ROS_INFO_STREAM("COORDS_3:" << (int)xpos-pt.easting << "::" << (int)ypos-pt.northing);
    }
    pub_message->publish(gps_msg);

    seq++;
  }
}

/*----------------------------------------------------------------------
 * callbackPcl()
 * Callback function for laser scanner pcl.
 *--------------------------------------------------------------------*/

void LaserTransform::callbackPcl(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl_out = *msg;

  new_pcl_filtered = true;
}

/*----------------------------------------------------------------------
 * callbackOdometryFiltered()
 * Callback function for filtered odometry.
 *--------------------------------------------------------------------*/

void LaserTransform::callbackOdometryFiltered(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (new_pcl_filtered == true) 
  {
    // transform pcl to laser position
    pcl_ros::transformPointCloud("/base_link", laser_pose,  pcl_out, pcl_out);

/*
    if (FULL_SENSOR_LOGFILE)
    {
      full_log << msg->header.stamp;
    }*/

    xpos = msg->pose.pose.position.x;
    ypos = msg->pose.pose.position.y;

    // publish transformed plc
    publish_new_pcl = true;
    new_pcl_filtered = false;
    publishPclMessage(pcl_pub);
  }
}

/*----------------------------------------------------------------------
 * connectedCallback()
 * Callback function for Tinkerforge ip connected
 *--------------------------------------------------------------------*/

void LaserTransform::connectedCallback(uint8_t connect_reason, void *user_data)
{
  LaserTransform *lt = (LaserTransform*) user_data;
  if (lt->is_imu_connected == false)
    ipcon_enumerate(&(lt->ipcon));
  return;
}

/*----------------------------------------------------------------------
 * enumerateCallback()
 * Callback function for Tinkerforge enumerate
 *--------------------------------------------------------------------*/

void LaserTransform::enumerateCallback(const char *uid, const char *connected_uid,
                  char position, uint8_t hardware_version[3],
                  uint8_t firmware_version[3], uint16_t device_identifier,
                  uint8_t enumeration_type, void *user_data)
{
  LaserTransform *lt = (LaserTransform*) user_data;

  if(enumeration_type == IPCON_ENUMERATION_TYPE_DISCONNECTED)
  {
    return;
  }

  // check if device is an imu
  if(device_identifier == IMU_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found IMU with UID:" << uid);
    // Create IMU device object
    imu_create(&(lt->imu), uid, &(lt->ipcon));
    imu_set_convergence_speed(&(lt->imu),lt->imu_convergence_speed);
    imu_leds_on(&(lt->imu));
    lt->is_imu_connected = true;
  }
  else if (device_identifier == GPS_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found GPS with UID:" << uid);
    // Create GPS device object
    gps_create(&(lt->gps), uid, &(lt->ipcon));
    lt->is_gps_connected = true;
  }
  else if (device_identifier == INDUSTRIAL_DIGITAL_IN_4_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found IDI4 with UID:" << uid);
    // Create IndustrialDigitalIn4 device object
    industrial_digital_in_4_create(&(lt->idi4), uid, &(lt->ipcon));

    // Register threshold reached callback to function cb_reached

    industrial_digital_in_4_register_callback(&(lt->idi4),
      INDUSTRIAL_DIGITAL_IN_4_CALLBACK_INTERRUPT,
      (void*)idi4Callback,
      lt);

    // set debounce period
    industrial_digital_in_4_set_debounce_period(&(lt->idi4),10);

    // Enable interrupt on pin 0
    industrial_digital_in_4_set_interrupt(&(lt->idi4), 1 << 0);

    lt->is_idi4_connected = true;
  }
  else if (device_identifier == DUAL_BUTTON_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found DualButton with UID:" << uid);
    dual_button_create(&(lt->db), uid, &(lt->ipcon));

    // Register state changed callback to function cb_state_changed
    dual_button_register_callback(&(lt->db),
                                  DUAL_BUTTON_CALLBACK_STATE_CHANGED,
                                  (void *)dbCallback,
                                  lt);
  }
}

/*----------------------------------------------------------------------
 * idi4Callback()
 * Callback function for Tinkerforge Industrial Digital In 4 Bricklet
 *--------------------------------------------------------------------*/

void LaserTransform::idi4Callback(uint8_t interrupt_mask, uint8_t value_mask, void *user_data)
{
  LaserTransform *lt = (LaserTransform*) user_data;
  static ros::Time begin = ros::Time(0,0);

  // check if correct input channel
  if (interrupt_mask != 0x1)
    return;

  // check for falling edge (ignore)
  if (interrupt_mask == 0x1 && value_mask == 0x0)
    return;

  if (begin.sec == 0)
    begin = ros::Time::now();
  else {
    ros::Time end = ros::Time::now();
    // calculate time between two impulses in ms
    float diff = (end.sec* 1000 + end.nsec / 1000000) - (begin.sec * 1000 + begin.nsec / 1000000);

    // check if vehicle is moving
    if (diff < 3000) 
    {
      // calculate rev
      lt->rev = (1000.0/diff)/2.0; // 2 magnets
      // ... and the velocity in m/s
      lt->velocity = (1.4207 * lt->rev - 0.0409)/3.6;

      if (lt->isMeasure == true)
        lt->velo_log << end <<  "::" << lt->rev << "::" << diff << std::endl;

      //ROS_INFO_STREAM("Zeit End:" << end.sec << "::" << end.nsec);
      //ROS_INFO_STREAM("Zeit Start:" << begin.sec << "::" << begin.nsec);
      //ROS_INFO_STREAM("Diff:" << diff << " ms");
      ROS_INFO_STREAM("Drehzahl:" << lt->rev << " #### Geschwindigkeit:" << (lt->velocity*3.6));
    } else
    {
      // ignore first rev value after stand still
      ROS_INFO_STREAM("START");
    }

    // reset timestamps
    begin = ros::Time::now();
    lt->last_rev = ros::Time::now();
  }
  return;
}

/*----------------------------------------------------------------------
 * dbCallback()
 * Callback function for Tinkerforge Dual Button Bricklet
 *--------------------------------------------------------------------*/

void LaserTransform::dbCallback(uint8_t button_l, uint8_t button_r,
                      uint8_t led_l, uint8_t led_r,
                      void *user_data)
{
  LaserTransform *lt = (LaserTransform*) user_data;
  (void)led_l; // avoid unused parameter warning
  (void)led_r; // avoid unused parameter warning
  (void)user_data; // avoid unused parameter warning

  if (VELO_LOGFILE == false)
    return;

  if(button_l == DUAL_BUTTON_BUTTON_STATE_PRESSED && lt->isMeasure == false)
  {
    lt->rpm_cnt = 0;
    std::stringstream ss;

    time_t t;
    struct tm *ts;
    char buff[80];

    // build filename
    t = time(NULL);
    ts = localtime(&t);

    strftime(buff, 80, "#####%Y_%m_%d-%H_%M_%S#####", ts);
    lt->velo_log << buff  << std::endl;
    lt->isMeasure = true;
  }

  if(button_r == DUAL_BUTTON_BUTTON_STATE_PRESSED)
  {
    lt->isMeasure = false;
  }
}

/*----------------------------------------------------------------------
 * getQuaternion()
 * get IMU quaternion
 *--------------------------------------------------------------------*/

tf::Quaternion LaserTransform::getQuaternion()
{
  float x = 0.0, y = 0.0, z = 0.0, w = 0.0;

  if (is_imu_connected)
    imu_get_quaternion(&imu, &x, &y, &z, &w);

  //-------
  float yaw   =  atan2(2.0*(x*y + w*z), pow(w,2)+pow(x,2)-pow(y,2)-pow(z,2));
  float pitch = -asin(2.0*(w*y - x*z));
  float roll  = -atan2(2.0*(y*z + w*x), -(pow(w,2)-pow(x,2)-pow(y,2)+pow(z,2)));
  std::cout << "y:" << rad2deg(yaw) << ":: p:" << rad2deg(pitch) << ":: r:" << rad2deg(roll) << std::endl;
  //-------

  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

/*----------------------------------------------------------------------
 * publishOdometryMessage()
 * Publish the odometry data (velocity vx)
 *--------------------------------------------------------------------*/

void LaserTransform::publishOdometryMessage(ros::Publisher *pub_message)
{
  static uint32_t seq = 0;
  static bool isStand = false;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time = ros::Time::now();

  // check if vehicle stand still, after 3 seconds without rev
  if (ros::Time::now().sec - last_rev.sec >= 3)
  {
    if (isStand == false)
    {
      ROS_INFO_STREAM("STAND");
      isStand = true;
    }
    velocity = 0.0;
  }
  else
  {
    isStand = false;
  }

  nav_msgs::Odometry odo_msg;

  // message header
  odo_msg.header.seq =  seq;
  odo_msg.header.stamp = current_time;
  odo_msg.header.frame_id = "odom";
  odo_msg.child_frame_id = "base_link";

  odo_msg.pose.pose.position.x = 0;
  odo_msg.pose.pose.position.y = 0;
  odo_msg.pose.pose.position.z = 0;

  odo_msg.twist.twist.linear.x = 1.0;
  odo_msg.twist.twist.angular.x = 0;

  // twist.covariance
  boost::array<const double, 36> tc =
    { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
      0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
      0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  odo_msg.twist.covariance = tc;

  // pose.covariance
  boost::array<const double, 36> pc =
    { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
      0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
      0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  odo_msg.pose.covariance = pc;

  pub_message->publish(odo_msg);

  seq++;

  return;
}

/*----------------------------------------------------------------------
 * setLaserPose()
 * Set the laser pose
 *--------------------------------------------------------------------*/
void LaserTransform::setLaserPose(double x, double y, double z,
  double yaw, double pitch, double roll)
{
  tf::Quaternion q;
  q.setRPY(deg2rad(roll),deg2rad(pitch),deg2rad(yaw));
  laser_pose.setOrigin(tf::Vector3(x, y, z));
  laser_pose.setRotation(q);
  //std::cout << x << "::" << y << "::" << z << "::" << yaw << "::" << pitch << "::" << roll << std::endl;
  return;
}

/*----------------------------------------------------------------------
 * getPosition()
 * get Position
 *--------------------------------------------------------------------*/

int LaserTransform::getPosition(float *x_pos, float *y_pos, float *z_pos)
{
  *x_pos = xpos;
  *y_pos = ypos;
  *z_pos = 0;
  return true;
}
