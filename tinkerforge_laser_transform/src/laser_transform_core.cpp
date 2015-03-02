#include <iostream>
#include "ros/ros.h"
#include "laser_transform_core.h"
#include "pcl_ros/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "ip_connection.h"
#include "brick_imu.h"
#include "bricklet_gps.h"

//#define HOST "141.56.161.43"
#define HOST 	"192.168.0.55"
#define PORT 	4223

#define M_PI	3.14159265358979323846  /* pi */

/*----------------------------------------------------------------------
 * LaserTransform()
 * Constructor
 *--------------------------------------------------------------------*/

LaserTransform::LaserTransform()
{
  publish_new_pcl = false;
  is_imu_connected = false;
  is_gps_connected = false;
}

/*----------------------------------------------------------------------
 * ~LaserTransform
 * Destrucot
 *--------------------------------------------------------------------*/

LaserTransform::~LaserTransform()
{
  if (is_imu_connected)
  {
    imu_leds_off(&imu);
    ipcon_destroy(&ipcon);
  }
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
 * publishMessage()
 * Publish the message.
 *--------------------------------------------------------------------*/

void LaserTransform::publishMessage(ros::Publisher *pub_message)
{
  if (publish_new_pcl)
  {
    ROS_INFO_STREAM("pcloud out");
    pub_message->publish(pcl_out);
    publish_new_pcl = false;
  }
}

/*----------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *--------------------------------------------------------------------*/

void LaserTransform::messageCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //ROS_INFO_STREAM("pcloud in");
  float xp, yp, zp;
  tf::Transform transform;
  getPosition(&xp, &yp, &zp);
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q = getQuaternion();
  tf::Quaternion q2;
  q2.setEulerZYX(0,0,-1.57079633);
  transform.setRotation(q2);
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
    imu_set_convergence_speed(&(lt->imu),0);
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
  
  tf::Quaternion q(x, y*-1, z, w);  
  return q;
}

/*----------------------------------------------------------------------
 * getPosition()
 * get Position
 *--------------------------------------------------------------------*/
 
int LaserTransform::getPosition(float *x_pos, float *y_pos, float *z_pos)
{
  return true;
}
