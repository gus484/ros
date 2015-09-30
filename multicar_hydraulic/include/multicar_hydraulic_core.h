#ifndef MULTICAR_HYDRAULIC_CORE_H_
#define MULTICAR_HYDRAULIC_CORE_H_

#include <string>
#include <stdint.h>
#include <stdio.h>
#include "multicar_hydraulic/CanCmd.h"
#include "tinycan/CanMsg.h"
#include "sensor_msgs/JointState.h"

namespace hydraulic {

// hydraulic pre control states
#define PRE_CONTROL_SIG_ON 1
#define PRE_CONTROL_SIG_ON_ACK 2
#define PRE_CONTROL_SIG_OFF 3
#define PRE_CONTROL_SIG_OFF_ACK 4

// hydraulic states
#define HYDRAULIC_HA_NA 0
#define HYDRAULIC_NA_VS 1
#define HYDRAULIC_HA_SWK 2

// hydraulic movements
#define MOVE_HALT 0
#define MOVE_MOVING 1
#define MOVE_THRUST_RIGHT 2
#define MOVE_THRUST_LEFT 3
#define MOVE_UP 2
#define MOVE_DOWN 3

// sensors
#define CO_NMT_MSG 0x00
#define CO_VALUES_MSG 0x180
#define CO_BOOM_MSG 0x280
#define CO_HEARTBEAT_MSG 0x700
#define CO_SYNC_MSG 0x80

// hydraulic
#define CO_HYDRAULIC_ACK 0x180
#define CO_PRE_CONTROL 0x200

#define NID_Hydraulik 20
#define NID_Schwenkfix 124 // only virtual
#define NID_Nebenarm 126
#define NID_Hauptarm 127
#define NID_THRUST 128
#define NID_Schnellwechselkopf 125 // only virtual

#define IDX_Nebenarm 1
#define IDX_Hauptarm 2
#define IDX_VERSCHUB 3
#define IDX_Schnellwechselkopf 4

struct Ausleger {
  static uint8_t pre_control_signal;
  uint16_t node_id;
  std::string node_name;
  uint8_t state;
  float position;
  int16_t velocity;
  uint32_t target_position;
  uint32_t target_velocity;
  bool is_moveing;
  void setValues(uint32_t position, int16_t speed)
  {
    this->position = position * 100; //  µm
    this->position = roundf(this->position / 1000.0); // µm to mm
    this->velocity = speed; // mm/s
  }
  void printValues()
  {
    printf("Node %d (%s) :: Position:%f mm :: Velocity:%d mm/s\n", node_id, 
      node_name.c_str(), position, velocity);
  }
};

class Hydraulic {

  public:
    //! The constructor
    Hydraulic();
    //! The destructor
    ~Hydraulic();
    //! Init
    int init();
    //! Publish can message.
    void publishCanMessage(tinycan::CanMsg *msg);
    //! Publisher JointState Message
    void publishJointStateMessage(int nid, uint32_t pos, uint16_t velo);
    //! Callback for can messages
    void callbackCanMessageRaw(const tinycan::CanMsg::ConstPtr& msg);
    //! Callback for moveit robot trajectory
    void callbackRobotTrajectory(const sensor_msgs::JointState::ConstPtr& msg);
    //! Handle hydraulic movement tasks
    int checkTask();
    //! Setter for CAN Message publisher
    void setCanMsgPublisher(ros::Publisher *p) { cpub = p; }
    //! Setter for JointState publisher
    void setJointStatePublisher(ros::Publisher *p) { jspub = p; }
    //! Build CAN Message from commando
    void cmdToCanMessage(std::string cmd, int val);
    //! Set PWM signal for selected node
    void setPWM(uint8_t nid, uint16_t pwm);
  private:
    Ausleger ausleger[5];
    uint8_t hydraulic_states[3][8];
    ros::Publisher *cpub;
    ros::Publisher *jspub;
    bool is_man_ctr;
    bool is_man_ctr_act;
};
}
#endif
