#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include "laser_transform_core.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <geometry_msgs/Point.h>
#include "ip_connection.h"
#include "brick_imu.h"
#include "bricklet_gps.h"
#include "bricklet_industrial_dual_0_20ma.h"

#define HOST "localhost"
//#define HOST "141.56.161.43"
//#define HOST 	"192.168.0.55"
#define PORT 	4223

#define DUAL_SENSOR1 0
#define DUAL_SENSOR2 1
#define DUAL_OPT INDUSTRIAL_DUAL_0_20MA_THRESHOLD_OPTION_GREATER
#define DUAL_MIN 10 * 1000000
#define DUAL_MAX 10 * 1000000
#define DUAL_NUM_OF_MAGNETS 1

#define GPS_LOGFILE true
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
  is_dual020_connected = false;
  start_latitude = 0;
  start_longitude = 0;
  velocity = 0.0;
  dual020_trigger_cnt_c1 = 0;
  dual020_trigger_cnt_c2 = 0;
  imu_convergence_speed = 0;

  xpos = ypos = 0;
  // set laser scanner orientation to imu orientation
  laser_orientation.setRPY(-90*M_PI/180.0,0,0);

  // open gps log file
  if (GPS_LOGFILE)
  {
    std::stringstream ss;

    time_t t;
    struct tm *ts;
    char buff[80];

    // build filename
    t = time(NULL);
    ts = localtime(&t);

    strftime(buff, 80, "gps_log_%Y_%m_%d-%H_%M_%S.txt", ts);
    ss << LOGFILE_PATH << buff;

    gps_log.open(ss.str().c_str(), std::ios::out);
  }
}

/*----------------------------------------------------------------------
 * ~LaserTransform
 * Destructor
 *--------------------------------------------------------------------*/

LaserTransform::~LaserTransform()
{
  if (is_imu_connected)
  {
    imu_leds_off(&imu);
    ipcon_destroy(&ipcon);
  }
  // close gps logfile
  if (gps_log.is_open())
    gps_log.close();
}

/*----------------------------------------------------------------------
 * Init()
 * Init the TF-Devices
 *--------------------------------------------------------------------*/

int LaserTransform::init()
{
  // create IP connection
  ipcon_create(&ipcon);

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

  // connect to brickd
  if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
    std::cout << "could not connect to brickd!" << std::endl;
    return false;
  }
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
 * odometryCallback()
 * Callback function for filtered odometry.
 *--------------------------------------------------------------------*/
void LaserTransform::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //msg.pose.pose.point.x
  return;
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
  if (is_imu_connected)
  {
    sensor_msgs::Imu imu_msg;

    imu_get_quaternion(&imu, &x, &y, &z, &w);

    imu_get_all_data(&imu, &acc_x, &acc_y, &acc_z, &mag_x, &mag_y,
      &mag_z, &ang_x, &ang_y, &ang_z, &temp);
    /*
    float yaw   =  atan2(2.0*(x*y + w*z), pow(w,2)+pow(x,2)-pow(y,2)-pow(z,2));
    float pitch = -asin(2.0*(w*y - x*z));
    float roll  = -atan2(2.0*(y*z + w*x), -(pow(w,2)-pow(x,2)-pow(y,2)+pow(z,2)));
1
    tf::Quaternion q;
    q.setEuler(yaw, pitch, roll);
    */
    // message header
    imu_msg.header.seq = seq;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "/world";
    // imu data
    /*
    imu_msg.orientation.x = q.getAxis()[0];
    imu_msg.orientation.y = q.getAxis()[1];
    imu_msg.orientation.z = q.getAxis()[2];
    imu_msg.orientation.w = q.getW();
    */
    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    imu_msg.orientation_covariance[0] = -1;

    imu_msg.angular_velocity.x = ang_x;
    imu_msg.angular_velocity.y = ang_y;
    imu_msg.angular_velocity.z = ang_z;
    imu_msg.angular_velocity_covariance[0] = -1;

    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;
    imu_msg.linear_acceleration_covariance[0] = -1;

    pub_message->publish(imu_msg);

    seq++;
  }
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

    sensor_msgs::NavSatFix gps_msg;

    // message header
    gps_msg.header.seq =  seq;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = "/world";
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
 * pclCallback()
 * Callback function for laser scanner pcl.
 *--------------------------------------------------------------------*/

void LaserTransform::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //ROS_INFO_STREAM("pcloud in");
  float xp, yp, zp;
  tf::Transform transform;
  getPosition(&xp, &yp, &zp);
  transform.setOrigin( tf::Vector3(xp, yp, zp) );
  tf::Quaternion q = getQuaternion();
  transform.setRotation(laser_orientation);
  sensor_msgs::PointCloud2 pcl_tmp;
  pcl_ros::transformPointCloud("/world2", transform,  *msg, pcl_tmp);
  transform.setRotation(q);
  pcl_ros::transformPointCloud("/world", transform,  pcl_tmp, pcl_out);
  publish_new_pcl = true;
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
    imu_set_convergence_speed(&(lt->imu),5);
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
  else if (device_identifier == INDUSTRIAL_DUAL_0_20MA_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found ID20MA with UID:" << uid);
    // Create IndustrialDual020mA device object
    industrial_dual_0_20ma_create(&(lt->dual020), uid, &(lt->ipcon));
    // Get threshold callbacks with a debounce time of 20ms
    industrial_dual_0_20ma_set_debounce_period(&(lt->dual020), lt->imu_convergence_speed);

    // Register threshold reached callback to function cb_reached

    industrial_dual_0_20ma_register_callback(&(lt->dual020),
      INDUSTRIAL_DUAL_0_20MA_CALLBACK_CURRENT_REACHED,
      (void*)dual020Callback,
      lt);

    industrial_dual_0_20ma_set_current_callback_threshold(&(lt->dual020),
      DUAL_SENSOR1, DUAL_OPT, DUAL_MIN, DUAL_MAX);
    industrial_dual_0_20ma_set_current_callback_threshold(&(lt->dual020),
      DUAL_SENSOR2, DUAL_OPT, DUAL_MIN, DUAL_MAX);

    // set sample rate for sensors
    industrial_dual_0_20ma_set_sample_rate(&(lt->dual020),
      INDUSTRIAL_DUAL_0_20MA_SAMPLE_RATE_240_SPS);
    lt->is_dual020_connected = true;
  }
}

/*----------------------------------------------------------------------
 * dual020Callback()
 * Callback function for Tinkerforge Industrial Dual 0-20mA Bricklet
 *--------------------------------------------------------------------*/

void LaserTransform::dual020Callback(uint8_t sensor, int32_t current, void *user_data)
{
  LaserTransform *lt = (LaserTransform*) user_data;
  static ros::Time begin = ros::Time::now();
  static int cnt_c1 = 0, cnt_c2 = 0;
  //ROS_INFO_STREAM(current);

  if (sensor == 0)
  {
    cnt_c1++;
    //lt->dual020_trigger_cnt_c1++;
  }
  else
  {
    cnt_c2++;
    //lt->dual020_trigger_cnt_c2++;
  }

  if ((ros::Time::now().sec - begin.sec) >= 1)
  {
    // calc rounds per minute
    //ROS_INFO_STREAM("Rounds per minute:" << lt->dual020_trigger_cnt / DUAL_NUM_OF_MAGNETS);
    ROS_INFO_STREAM("Sensor 1:" << cnt_c1 / DUAL_NUM_OF_MAGNETS << " U/Sek" << "::" << cnt_c1);
    ROS_INFO_STREAM("Sensor 2:" << cnt_c2 / DUAL_NUM_OF_MAGNETS << " U/Sek" << "::" << cnt_c2);

    //lt->dual020_trigger_cnt_c1 = 0;
    //lt->dual020_trigger_cnt_c2 = 0;
	cnt_c1 = cnt_c2 = 0;
    begin = ros::Time::now();
  }
  return;
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
  q.setEuler(yaw, pitch, roll);
  //tf::Quaternion q(x, y, z, w);
  //tf::Quaternion q(x, y*-1, z, w);
  return q;
}

/*----------------------------------------------------------------------
 * getVelocity()
 * get Velocity
 *--------------------------------------------------------------------*/

int LaserTransform::getVelocity(float *velocity)
{
  *velocity = 0.0;
  return true;
}

/*----------------------------------------------------------------------
 * getPosition()
 * get Position
 *--------------------------------------------------------------------*/

int LaserTransform::getPosition(float *x_pos, float *y_pos, float *z_pos)
{
  *x_pos = 0;
  *y_pos = 0;
  *z_pos = 0;
  return true;
}
