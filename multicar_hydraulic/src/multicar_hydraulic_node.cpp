#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tinycan/CanMsg.h"
#include "multicar_hydraulic_core.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/JointState.h"

using std::string;

int main (int argc, char **argv)
{
  hydraulic::Hydraulic *hydev = new hydraulic::Hydraulic();
  int rate = 10;
  string pub_topic_raw;
  string pub_topic_js;
  string sub_topic_raw;
  string sub_topic_robot_trajectory;

  ros::init(argc, argv, "multicar_hydraulic");

  ros::NodeHandle n;

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(50));
  private_node_handle_.param("pub_topic_raw", pub_topic_raw, string("/can_raw_send"));
  private_node_handle_.param("sub_topic_raw", sub_topic_raw, string("/can_raw_receive"));
  private_node_handle_.param("sub_topic_robot_trajectory", sub_topic_robot_trajectory, string("/move_group/fake_controller_joint_states"));
  private_node_handle_.param("pub_topic_jointstate", pub_topic_js, string("/joint_states"));

  // Create a publisher and name the topic
  ros::Publisher pub_can_message = n.advertise<tinycan::CanMsg>(pub_topic_raw.c_str(), 50);
  // Create a publisher for joint state messages
  ros::Publisher pub_joint_state_message = n.advertise<sensor_msgs::JointState>(pub_topic_js.c_str(), 50);
  // Create a subscriber for moveit robot trajectory
  ros::Subscriber sub_robot_trajectory = n.subscribe(sub_topic_robot_trajectory.c_str(), 1000, &hydraulic::Hydraulic::callbackRobotTrajectory, hydev);
  // Create a subscriber can raw messages
  ros::Subscriber sub_raw_message = n.subscribe(sub_topic_raw.c_str(), 1000, &hydraulic::Hydraulic::callbackCanMessageRaw, hydev);

  hydev->init();

  // set can publisher
  hydev->setCanMsgPublisher(&pub_can_message);
  hydev->setJointStatePublisher(&pub_joint_state_message);
  // start sensors
  ros::Duration(1.5).sleep();
  hydev->cmdToCanMessage("boot",0x7F);
  hydev->cmdToCanMessage("boot",0x7D);
  hydev->cmdToCanMessage("boot",0x7E);

  ros::Rate loop_rate(rate);

  while (ros::ok()) {
    // check for cbs
    hydev->checkTask();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
