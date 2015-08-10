#ifndef CAN_CORE_H_
#define CAN_CORE_H_

#include <string>
#include <stdint.h>
#include <stdio.h>
#include "can_drv.h"
#include "tinycan/CanMsg.h"

namespace tinycan {


class Can {

public:
  Can();
  ~Can();
  int init();
  //! Send a can message
  int sendMsg(TCanMsg *msg);
  //! Read a can message from bus
  int readMsg(TCanMsg *msg);
  //! Build a message
  int buildMsg(TCanMsg *msg, uint8_t id, uint8_t len, char* data);
  //! Print a message
  int printMsg(TCanMsg *msg);
  //! Publish can message.
  void publishCanMessage(ros::Publisher *pub_message, TCanMsg *msg);
  //! Callcack function for can subscriber.
  void callbackCanMessage(const tinycan::CanMsg::ConstPtr& msg);
  private:
    uint32_t device_idx;
};
}
#endif
