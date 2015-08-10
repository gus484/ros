#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tinycan/CanMsg.h"
#include "can_drv.h"
#include "can_core.h"

using std::string;

int main (int argc, char **argv)
{
  TCanMsg can_msg; // CanAPI-Message
  tinycan::Can *can_dev = new tinycan::Can();

  string pub_topic_raw;
  string sub_topic_raw;

  if (!can_dev->init()) {
    return 0;
  }

  ros::init(argc, argv, "can");

  ros::NodeHandle n;

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("pub_topic_raw", pub_topic_raw, string("/can_raw_receive"));
  private_node_handle_.param("sub_topic_raw", sub_topic_raw, string("/can_raw_send"));

  // create a publisher for tinycan::CanMsg
  ros::Publisher pub_can_message = n.advertise<tinycan::CanMsg>(pub_topic_raw.c_str(), 50);
  // create a subscriber for tinycan::CanMsg
  ros::Subscriber sub_message = n.subscribe(sub_topic_raw.c_str(), 1000, &tinycan::Can::callbackCanMessage, can_dev);

  ros::Rate loop_rate(50);
  int count = 0;

  while (ros::ok()) {
    // read can messages
    while (can_dev->readMsg(&can_msg)) {
      //can_dev->printMsg(&can_msg);
      can_dev->publishCanMessage(&pub_can_message, &can_msg);
    }

    // check for cbs
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
