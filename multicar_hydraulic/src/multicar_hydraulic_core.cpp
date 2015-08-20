#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdio>
#include <stdint.h>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "multicar_hydraulic_core.h"
#include "multicar_hydraulic/CanCmd.h"
#include "tinycan/CanMsg.h"
#include "sensor_msgs/JointState.h"

#define DEBUG_CAN_OUTPUT false

namespace hydraulic {

uint8_t Ausleger::pre_control_signal = PRE_CONTROL_SIG_OFF_ACK;

bool isBetween(uint32_t num, uint32_t lower, uint32_t upper)
{
  if (num >= lower && num <= upper)
    return true;
  else
    return false;
}

Hydraulic::Hydraulic()
{
  return;
}

Hydraulic::~Hydraulic()
{

}

int Hydraulic::init()
{
  std::string names[5] = {"Schnellwechselkopf", "Nebenarm", "Hauptarm", "Verschub", "Schwenkfix"};
  // init Ausleger
  for (int i = 0 ; i < 5 ; i++)
  {
    ausleger[i].node_id = i + 125;
    ausleger[i].node_name = names[i];
    ausleger[i].state = MOVE_HALT;
    ausleger[i].setValues(0,0);
    ausleger[i].target_position = 0;
    ausleger[i].target_velocity = 0;
  }
  cpub = NULL;
}

/*----------------------------------------------------------------------
 * publishCanMessage()
 * Publish can message.
 *--------------------------------------------------------------------*/

void Hydraulic::publishCanMessage(tinycan::CanMsg *msg)
{
  if (cpub != NULL)
    cpub->publish(*msg);
}

/*----------------------------------------------------------------------
 * publishJointState()
 * Publisher JointState Message
 *--------------------------------------------------------------------*/

void Hydraulic::publishJointStateMessage(int nid, uint32_t pos, uint16_t velo)
{
  if (jspub == NULL)
    return;

  static uint32_t seq = 0;
  ros::Time current_time = ros::Time::now();
  sensor_msgs::JointState js_msg;
  std::vector<std::string> s;
  std::vector<double> p;
  std::vector<double> v;
  std::vector<double> e;

  v.push_back(velo/1000.0); // from mm/s to m/s

  js_msg.header.seq = seq;
  js_msg.header.stamp = current_time;
  js_msg.header.frame_id = "odom";

  switch(nid)
  {
    case NID_THRUST:
      if (pos > 612)
        p.push_back((pos-612)/1000.0*-1);
      else
        p.push_back((612-pos)/1000.0);
      s.push_back("trans_schlitten");
      e.push_back(0);
    break;
    case NID_Hauptarm: // 46.450000 cm
      p.push_back(-37699.0/8246000.0*pos + 2321764533.0/412300000.0);
      s.push_back("rot_dreharm_hauptarm");
      e.push_back(1000);
    break;
    case NID_Nebenarm: // max. 29cm
      p.push_back(-5.0/797.0*pos+ 469937.0/79700.0);
      s.push_back("rot_hauptarm_nebenarm");
      e.push_back(1000);
    break;
    case NID_Schnellwechselkopf: // TODO nicht getestet!!!
      p.push_back((ceilf(59.0/4680.0*pos + 5777.0/4680.0)*100)/ 100);
      s.push_back("rot_nebenarm_schnellwechselsystem");
      e.push_back(1000);
    break;
  }
  js_msg.position = p;
  js_msg.velocity = v;

  js_msg.name = s;
  js_msg.effort = e;

  jspub->publish(js_msg);
  seq++;
  return;
}

/*----------------------------------------------------------------------
 * callbackRobotTrajectory()
 * Callback for robot trajectory.
 *--------------------------------------------------------------------*/

void Hydraulic::callbackRobotTrajectory(const sensor_msgs::JointState::ConstPtr& msg) //trajectory_msgs::JointTrajectory
{
  uint16_t p = 0;
  for (unsigned int i=0; i< msg->name.size(); i++)
  {
    std::cout << msg->name[i] << std::endl;
    // Verschub
    if (msg->name[i].compare(std::string("trans_schlitten")) == 0)
    {
      std::cout << "    "  << "Position (ROS):" << msg->position[i] << std::endl;
      p = msg->position[i] * 1000; // convert from m to mm
      if (p > 0)
        p = 612-p;
      else
        p = 612 + (-1*p);
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      ausleger[IDX_VERSCHUB].target_position = p;
      ausleger[IDX_VERSCHUB].is_moveing = false;
      if ( ausleger[IDX_VERSCHUB].position < ausleger[IDX_VERSCHUB].target_position)
        ausleger[IDX_VERSCHUB].state = MOVE_THRUST_RIGHT;
      else
        ausleger[IDX_VERSCHUB].state = MOVE_THRUST_LEFT;
    }
    // Hauptarm
    else if (msg->name[i].compare(std::string("rot_dreharm_hauptarm")) == 0)
    {
      std::cout << "    "  << msg->position[i] << std::endl;
      // convert from rad to mm
      p = (msg->position[i] - 2321764533.0/412300000.0) / (-37699.0/8246000.0);
      std::cout << "    "  << "Position (ROS):" << msg->position[i] << std::endl;
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      ausleger[NID_Hauptarm].target_position = p;
      ausleger[NID_Hauptarm].is_moveing = false;
      if (p < ausleger[NID_Hauptarm].position)
      {
        ausleger[NID_Hauptarm].state = MOVE_DOWN;
      }
      else
      {
        ausleger[NID_Hauptarm].state = MOVE_UP;
      }
    }
    // Nebenarm
    else if (msg->name[i].compare(std::string("rot_dreharm_nebenarm")) == 0)
    {
      std::cout << "    "  << msg->position[i] << std::endl;
    }
  }
  return;
}

/*----------------------------------------------------------------------
 * canMsgRawCallback()
 * Callback for can messages.
 *--------------------------------------------------------------------*/

void Hydraulic::callbackCanMessageRaw(const tinycan::CanMsg::ConstPtr& msg)
{
  uint16_t cid = 0;
  uint8_t node_id = 0;
  //cid = msg->id >> 7;
  cid = msg->id & 0x0000FF80;
  node_id = msg->id & 0x0000007F;

  if (node_id == NID_Hydraulik)
  {
    switch(cid)
    {
      case CO_HYDRAULIC_ACK:
        if (msg->data[1] == 0x40)
        {
          Ausleger::pre_control_signal = PRE_CONTROL_SIG_ON_ACK;
          printf("pre sig on sck\n"); 
        }
        if (msg->data[1] == 0x00)
        {
          Ausleger::pre_control_signal = PRE_CONTROL_SIG_OFF_ACK;
          printf("pre sig off sck\n"); 
        }
      break;
      case CO_BOOM_MSG:
      {
        uint16_t Schnellwechselkopf = 0, thrust = 0, main_pressure = 0, pivoting = 0;
        Schnellwechselkopf = msg->data[1] + (msg->data[0] << 8);
        thrust = msg->data[2] + (msg->data[3] << 8);
        main_pressure = msg->data[4] + (msg->data[5] << 8);
        pivoting = msg->data[6] + (msg->data[7] << 8);
        if (DEBUG_CAN_OUTPUT) printf("Verschub:%d :: Druck Hauptarm:%d :: Schwenken:%d\n", thrust, main_pressure, pivoting);
        // calc position (pos = 0.120 * thrust - 27.200)
        if (0.120 * thrust - 27.200 <= 0)
          ausleger[IDX_VERSCHUB].position = 0;
        else
          ausleger[IDX_VERSCHUB].position = 0.120 * thrust - 27.200; // in mm
        ausleger[IDX_Schnellwechselkopf].position = Schnellwechselkopf;
        //std::cout << "Wert:" << ausleger[IDX_VERSCHUB].position << std::endl;
        publishJointStateMessage(NID_THRUST, ausleger[IDX_VERSCHUB].position, 0);
        publishJointStateMessage(NID_Schnellwechselkopf, ausleger[IDX_Schnellwechselkopf].position, 0);
      }
      break;
      default:

      break;
    }
  }
  else if (node_id >= 125 && node_id <= 127)
  {
    switch(cid)
    {
      case CO_VALUES_MSG:
      {
        int32_t pos = msg->data[0] + (msg->data[1] << 8) + (msg->data[2] << 16) + (msg->data[3] << 24);
        int16_t v = msg->data[4] + (msg->data[5] << 8);
        ausleger[node_id-125].setValues(pos,v);
        if (DEBUG_CAN_OUTPUT)  ausleger[node_id-125].printValues();
        publishJointStateMessage(node_id, ausleger[node_id-125].position, ausleger[node_id-125].velocity);
      }
      break;
      case CO_HEARTBEAT_MSG:
        if (DEBUG_CAN_OUTPUT) printf("Heartbeat from Node:%d :: State:%X\n", node_id, msg->data[0]);
      break;
      default:
        if (DEBUG_CAN_OUTPUT) printf("other message from device:%d :: %d\n", msg->id, node_id);
      break;
    }
  }
  else
  {
    switch(cid)
    {
      case CO_NMT_MSG:
        if (msg->data[0] == 0x01)
          if (DEBUG_CAN_OUTPUT) printf("NMT start indicate to node:%X\n", msg->data[1]);
      break;
      case CO_HEARTBEAT_MSG:
        if (DEBUG_CAN_OUTPUT) printf("Heartbeat from Node %X, State: %d\n", node_id, msg->data[0]);
      break;
      case 0x80:
        if (node_id == 0x00)
          if (DEBUG_CAN_OUTPUT) printf("Sync Message\n");
      break;
      case 0x180:
        if (DEBUG_CAN_OUTPUT) printf("TxPDO1 Message from Node %X\n", node_id);
      break;
      case 0x280:
        if (DEBUG_CAN_OUTPUT) printf("TxPDO2 Message from Node %X\n", node_id);
      break;
      case 0x580:
        if (DEBUG_CAN_OUTPUT) printf("SDO Response from Node %X\n",node_id);
      break;
      case 0x600:
        if (DEBUG_CAN_OUTPUT) printf("SDO Request from Node %X\n",node_id);
      break;
      default:
        ;
        if (DEBUG_CAN_OUTPUT) printf("message from unknown node:%X :: %X\n", node_id, msg->id);
      break;
    }
  }
}

/*----------------------------------------------------------------------
 * cmdToCanMessage()
 *
 *--------------------------------------------------------------------*/

void Hydraulic::cmdToCanMessage(std::string cmd, int val = 0)
{
  tinycan::CanMsg cmsg;
  cmsg.id = 0x000;
  cmsg.len = 8;
  for (uint8_t i = 0; i < 8 ; i++)
    cmsg.data[i] = 0;
  //std::cout << cmd << std::endl;
  if (cmd.compare("boot") == 0)
  {
    cmsg.id = 0x000;
    cmsg.len = 2;
    cmsg.data[0] = 0x01;
    cmsg.data[1] = val;
    //std::cout << cmsg.len << std::endl;
  } else if (cmd.compare("preop") == 0)
  {
    cmsg.id = 0x000;
    cmsg.len = 2;
    cmsg.data[0] = 0x80;
    cmsg.data[1] = 0x00;
  } else if (cmd.compare("sync") == 0)
  {
    cmsg.id = 0x080;
    cmsg.len = 0;
  } else if (cmd.compare("pcs_on") == 0) // activate pre control signal
  {
    std::cout << "pcs_on" << std::endl;
    cmsg.id = 0x214;
    cmsg.len = 3;
    cmsg.data[1] = 0x40;
  } else if (cmd.compare("pcs_off") == 0) // deactivate pre control signal
  {
    cmsg.id = 0x214;
    cmsg.len = 3;
  }
  cpub->publish(cmsg);
}

/*----------------------------------------------------------------------
 * setPWM()
 * Set PWM signal for selected node.
 *--------------------------------------------------------------------*/
void Hydraulic::setPWM(uint8_t nid, uint16_t pwm)
{
  tinycan::CanMsg cmsg;
  for (uint8_t i = 0 ; i < 8 ; i++)
    cmsg.data[i] = 0x00;
  switch(nid)
  {
    case NID_THRUST:
      if (pwm == 0) // stop moveing
      {
        // set pwm value 0
        cmsg.id = 0x514;
        cmsg.len = 8;
        cmsg.data[0] = cmsg.data[1] = cmsg.data[2] = cmsg.data[3] = 0x00;
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
        
        // deactivate pwm
        cmsg.id = 0x314;
        cmsg.len = 2;
        cmsg.data[0] = 0x00; // pwm thrust left on
        cmsg.data[1] = 0x02;
        publishCanMessage(&cmsg);
        break;
      }
      if (ausleger[NID_THRUST].state == MOVE_THRUST_LEFT)
      {
        // activate pwm left
        cmsg.id = 0x314;
        cmsg.len = 2;
        cmsg.data[0] = 0x10; // pwm thrust left on
        cmsg.data[1] = 0x02;
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        cmsg.id = 0x514;
        cmsg.len = 8;
        cmsg.data[0] = pwm & 0x00FF;
        cmsg.data[1] = pwm >> 8;
        publishCanMessage(&cmsg);
      }
      else
      {
        // activate pwm right
        cmsg.id = 0x314;
        cmsg.len = 2;
        cmsg.data[0] = 0x20; // pwm thrust right on
        cmsg.data[1] = 0x02;
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        cmsg.id = 0x514;
        cmsg.len = 8;
        cmsg.data[2] = pwm & 0x00FF;
        cmsg.data[3] = pwm >> 8;
        publishCanMessage(&cmsg);
      }
    break;
    case NID_Hauptarm:
      if (pwm == 0) // stop moveing
      {
        // set pwm value 0
        cmsg.id = 0x394;
        cmsg.len = 8;
        cmsg.data[0] = cmsg.data[1] = 0;
        publishCanMessage(&cmsg);
        break;
      }
      if (ausleger[NID_Hauptarm].state == MOVE_UP)
      {
        // activate pwm up
        cmsg.id = 0x394;
        cmsg.len = 8;
        cmsg.data[2] = pwm & 0x00FF;
        cmsg.data[3] = pwm >> 8;
        publishCanMessage(&cmsg);
      }
      else
      {
        // activate pwm down
        cmsg.id = 0x394;
        cmsg.len = 8;
        cmsg.data[0] = pwm & 0x00FF;
        cmsg.data[1] = pwm >> 8;
        publishCanMessage(&cmsg);
      }
    break;
  }
}

/*----------------------------------------------------------------------
 * checkTask()
 * Handle hydraulic movement tasks.
 *--------------------------------------------------------------------*/

int Hydraulic::checkTask()
{
  // check if waiting for pre-control singal ack
  if (Ausleger::pre_control_signal == PRE_CONTROL_SIG_ON ||
      Ausleger::pre_control_signal == PRE_CONTROL_SIG_OFF)
  {
     return 0;
  }

  // check for new movement
  for (uint8_t i = 0 ; i < 5 ; i++)
  {
    if (ausleger[i].is_moveing == false && ausleger[i].state != MOVE_HALT)
    {
      // check if pre control signal is active
      if (Ausleger::pre_control_signal == PRE_CONTROL_SIG_OFF_ACK)
      { // not active
        //set pcs on and waiting for ack
        cmdToCanMessage("pcs_on");
        Ausleger::pre_control_signal = PRE_CONTROL_SIG_ON;
        return 0;
      }
      else
      {
        ROS_INFO_STREAM("New movement for node " << ausleger[i].node_name);
        // set pwm
        setPWM(ausleger[i].node_id, 1250);

        ausleger[i].is_moveing = true;
        //ausleger[i].state = MOVE_MOVING;
      }
    }
  }

  // check for stopping movement or changing
  for (uint8_t i = 0 ; i < 5 ; i++)
  {
    if (ausleger[i].is_moveing == true && ausleger[i].state >= MOVE_MOVING)
    {
      ROS_INFO_STREAM("Check movement for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
      // check target
      if (ausleger[i].state == MOVE_THRUST_LEFT)
      {
        if (ausleger[i].position <= ausleger[i].target_position)
        {
          setPWM(ausleger[i].node_id,0); // stope Bewegung
          ROS_INFO_STREAM("Check movement l for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
          ausleger[i].is_moveing = false;
          ausleger[i].state = MOVE_HALT;
        }
      }
      if (ausleger[i].state == MOVE_THRUST_RIGHT)
      {
        if (ausleger[i].position >= ausleger[i].target_position)
        {
          setPWM(ausleger[i].node_id,0); // stope Bewegung
          ROS_INFO_STREAM("Check movement r for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
          ausleger[i].is_moveing = false;
          ausleger[i].state = MOVE_HALT;
        }
      }
    }
    /*
    else if (ausleger[i].is_moveing == true && ausleger[i].state > MOVE_MOVING)
    {
      ausleger[i].state = MOVE_MOVING;
    }
    */
  }

  // check for any active movement
  for (uint8_t i = 0; i < 5 ; i++)
  {
    if (ausleger[i].is_moveing == true)
      return 0;
  }
  
  // set pcs off and wait for ack
  cmdToCanMessage("pcs_off");
  Ausleger::pre_control_signal = PRE_CONTROL_SIG_OFF;
}

}
