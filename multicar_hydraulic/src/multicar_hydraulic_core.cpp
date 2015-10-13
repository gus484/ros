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

#define DEBUG_CAN_OUTPUT true
#define ALLOW_SWK true
#define ALLOW_SF false

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
    ausleger[i].is_moveing = false;
    ausleger[i].setValues(0,0);
    ausleger[i].target_position = 0;
    ausleger[i].target_velocity = 0;
  }
  // init hydraulic states
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    memset(hydraulic_states[i], 0x00, 8);
  }

  is_man_ctr = false;
  is_man_ctr_act = false;

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
  double tmp = 0.0;

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
    case NID_Hauptarm: // hauptarm.ods
      tmp = -0.003853211*pos + 5.4794495413;
      if (tmp < 3.68) tmp = 3.68;
      if (tmp > 4.94) tmp = 4.94;
      p.push_back(tmp);
      s.push_back("rot_dreharm_hauptarm");
      e.push_back(1000);
    break;
    case NID_Nebenarm: // nebenarm.ods
      tmp = -0.0086239216 * pos + 6.1126003922;
      if (tmp < 3.51) tmp = 3.51;
      if (tmp > 5.72) tmp = 5.72;
      p.push_back(tmp);
      s.push_back("rot_hauptarm_nebenarm");
      e.push_back(1000);
    break;
    case NID_Schnellwechselkopf: // swk.ods
      tmp = 0.0113780769 * pos + 1.2488742308;
      if (tmp < 1.80) tmp = 1.80;
      if (tmp > 4.76) tmp = 4.76;
      p.push_back(tmp);
      s.push_back("rot_nebenarm_schnellwechselsystem");
      e.push_back(1000);
    break;
    case NID_Schwenkfix:
      tmp = 0.0079487179*pos + 4.2133;
      // check if state is outside bounds by a significant margin
      if (tmp > 6.28318)
        tmp = 6.28318;
      else if ( tmp < 4.7298)
        tmp = 4.7298;
      p.push_back(tmp);
      //std::cout << "::" << (0.0079487179*pos + 4.2133) << std::endl;
      s.push_back("rot_schnellwechselsystem_maehkopf");
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
      std::cout << "    " << "Velocity soll (real):" << msg->velocity[i] << "m/s" << std::endl;
      std::cout << "    " << "Position (ROS):" << msg->position[i] << std::endl;
      p = msg->position[i] * 1000; // convert from m to mm
      if (p > 0)
        p = 612-p;
      else
        p = 612 + (-1*p);
      std::cout << "    " << "Position soll (real):" << p << "mm" << std::endl;
      std::cout << "    " << "Position ist  (real):" << ausleger[IDX_VERSCHUB].position << "mm" << std::endl;
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
      // convert from rad to mm hauptarm.ods
      p = -259.5238095238 * msg->position[i] + 1422.0476190476;
      if (p > 467.0) p = 467.0;
      if (p < 140.0) p = 140.0;
      std::cout << "    " << "Velocity soll (real):" << msg->velocity[i] << "m/s" << std::endl;
      std::cout << "    " << "Position (ROS):" << msg->position[i] << std::endl;
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      std::cout << "    " << "Position ist (real):" << ausleger[IDX_Hauptarm].position << "mm" << std::endl;
      ausleger[IDX_Hauptarm].target_position = p;
      ausleger[IDX_Hauptarm].is_moveing = false;
      if (p < ausleger[IDX_Hauptarm].position)
      {
        ausleger[IDX_Hauptarm].state = MOVE_DOWN;
      }
      else
      {
        ausleger[IDX_Hauptarm].state = MOVE_UP;
      }
    }
    // Nebenarm
    else if (msg->name[i].compare(std::string("rot_hauptarm_nebenarm")) == 0)
    {
      // convert from rad to mm nebenarm.ods
      p = (-115.9565276704 * msg->position[i] + 708.7959165113);
      if (p > 301.0) p = 301.0;
      if (p <  46.0) p =  46.0;
      std::cout << "    " << "Velocity soll (real):" << msg->velocity[i] << "m/s" << std::endl;
      std::cout << "    " << "Position (ROS):" << msg->position[i] << std::endl;
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      std::cout << "    " << "Position ist (real):" << ausleger[IDX_Nebenarm].position << "mm" << std::endl;
      // Workaround
      if (!is_man_ctr)
		setPWM(NID_Nebenarm, 0);
      // -----
      ausleger[IDX_Nebenarm].target_position = p;
      ausleger[IDX_Nebenarm].is_moveing = false;
      if (p < ausleger[IDX_Nebenarm].position)
      {
        ausleger[IDX_Nebenarm].state = MOVE_DOWN;
      }
      else
      {
        ausleger[IDX_Nebenarm].state = MOVE_UP;
      }
    }
    // Schnellwechselkopf
    else if (msg->name[i].compare(std::string("rot_nebenarm_schnellwechselsystem")) == 0)
    {
      // convert from rad to mm swk.ods
      p = (87.8883142345 * msg->position[i] - 109.7614508332);
      if (p > 309.0) p = 309.0;
      if (p <  49.0) p =  49.0;
      std::cout << "    " << "Velocity soll (real):" << msg->velocity[i] << "m/s" << std::endl;
      std::cout << "    " << "Position (ROS):" << msg->position[i] << std::endl;
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      std::cout << "    " << "Position ist (real):" << ausleger[IDX_Schnellwechselkopf].position << "mm" << std::endl;
      if (ALLOW_SWK)
      {
        // Workaround
        if (!is_man_ctr)
	  setPWM(NID_Schnellwechselkopf, 0);
        // -----
        ausleger[IDX_Schnellwechselkopf].target_position = p;
        ausleger[IDX_Schnellwechselkopf].is_moveing = false;
        if (p < ausleger[IDX_Schnellwechselkopf].position)
        {
          ausleger[IDX_Schnellwechselkopf].state = MOVE_DOWN;
        }
        else
        {
          ausleger[IDX_Schnellwechselkopf].state = MOVE_UP;
        }
      }
    }
    // Schwenkfix
    else if (msg->name[i].compare(std::string("rot_schnellwechselsystem_maehkopf")) == 0)
    {
      // convert from rad to mm
      p = (125.8064516129 * msg->position[i] - 530.064516129);
      std::cout << "    " << "Velocity soll (real):" << msg->velocity[i] << "m/s" << std::endl;
      std::cout << "    " << "Position (ROS):" << msg->position[i] << std::endl;
      std::cout << "    " << "Position (real):" << p << "mm" << std::endl;
      std::cout << "    " << "Position ist (real):" << ausleger[IDX_Schwenkfix].position << "mm" << std::endl;
      if (ALLOW_SF)
      {
        ausleger[IDX_Schwenkfix].target_position = p;
        ausleger[IDX_Schwenkfix].is_moveing = false;
        if (p < ausleger[IDX_Schwenkfix].position) // TODO: check direction
        {
          ausleger[IDX_Schwenkfix].state = MOVE_DOWN;
        }
        else
        {
          ausleger[IDX_Schwenkfix].state = MOVE_UP;
        }
      }
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
          hydraulic_states[HYDRAULIC_HA_SWK][1] = 0x40;
          Ausleger::pre_control_signal = PRE_CONTROL_SIG_ON_ACK;
          printf("pre sig on ack\n");
          if (is_man_ctr)
          {
			if (!is_man_ctr_act)
			{  
			  is_man_ctr_act = true;
		      setPWM(NID_Hauptarm, 0);
		      setPWM(NID_Nebenarm, 0);
		      setPWM(NID_THRUST, 0);
		    }
		  }        
        }
        if (msg->data[1] == 0x00)
        {
          hydraulic_states[HYDRAULIC_HA_SWK][1] = 0x00;
          Ausleger::pre_control_signal = PRE_CONTROL_SIG_OFF_ACK;
          printf("pre sig off ack\n"); 
        }
      break;
      case CO_PRE_CONTROL:
		printf("manuelle Kontrolle!\n");
		is_man_ctr = true;
      break;
      case CO_BOOM_MSG:
      {
        uint16_t Schnellwechselkopf = 0, thrust = 0, main_pressure = 0, pivoting = 0;
        pivoting = msg->data[0] + (msg->data[1] << 8);
        thrust = msg->data[2] + (msg->data[3] << 8);
        main_pressure = msg->data[4] + (msg->data[5] << 8);        
        // calc position (pos = 0.120 * thrust - 27.200)
        if (0.120 * thrust - 27.200 <= 0)
          ausleger[IDX_VERSCHUB].position = 0;
        else
          ausleger[IDX_VERSCHUB].position = 0.120 * thrust - 27.200; // in mm
        
        ausleger[IDX_Schwenkfix].position = 0.039 * pivoting - 97.101; // in mm
        //if (ausleger[NID_Schwenkfix].position < 0)
		//  ausleger[NID_Schwenkfix].position = 
        if (DEBUG_CAN_OUTPUT) printf("Verschub:%lf :: Druck Hauptarm:%d :: Schwenken:%lf\n", ausleger[IDX_VERSCHUB].position, main_pressure, ausleger[IDX_Schwenkfix].position);
        publishJointStateMessage(NID_THRUST, ausleger[IDX_VERSCHUB].position, 0);
        publishJointStateMessage(NID_Schwenkfix, ausleger[IDX_Schwenkfix].position, 0);
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
	std::cout << "pcs_off" << std::endl;  
    cmsg.id = 0x214;
    cmsg.len = 3;
    cmsg.data[1] = 0x00;
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
        //cmsg.data[0] = 0x00; // pwm thrust left on
        //cmsg.data[1] = 0x02;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] & 0x0C;
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;       
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8); 
        publishCanMessage(&cmsg);
        break;
      }
      if (ausleger[IDX_VERSCHUB].state == MOVE_THRUST_LEFT)
      {
        // activate pwm left
        cmsg.id = 0x314;
        cmsg.len = 2;
        //cmsg.data[0] = 0x10; // pwm thrust left on
        //cmsg.data[1] = 0x02;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] | 0x10;
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;        
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8);
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
        //cmsg.data[0] = 0x20; // pwm thrust right on
        //cmsg.data[1] = 0x02;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] | 0x20;
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8);    
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
        // deactivate hauptarm
        cmsg.id = 0x214;
        cmsg.len = 3;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] & 0xC0;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,3);
        
        // set pwm value 0
        cmsg.id = 0x394;
        cmsg.len = 8;
        for (uint8_t i = 0 ; i < 4 ; i++)
          hydraulic_states[HYDRAULIC_HA_NA][i] = 0;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
        break;
      }
      if (ausleger[IDX_Hauptarm].state == MOVE_UP)
      {
        // activate hauptarm up
        cmsg.id = 0x214;
        cmsg.len = 3;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] | 0x02; //0xC2
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,3);

        // activate pwm up
        cmsg.id = 0x394;
        cmsg.len = 8;
        hydraulic_states[HYDRAULIC_HA_NA][2] = pwm & 0x00FF;
        hydraulic_states[HYDRAULIC_HA_NA][3] = pwm >> 8;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
      }
      else
      {
        // activate hauptarm up
        cmsg.id = 0x214;
        cmsg.len = 3;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] | 0x01; //0xC1
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,3);

        // activate pwm down
        cmsg.id = 0x394;
        cmsg.len = 8;
        hydraulic_states[HYDRAULIC_HA_NA][0] = pwm & 0x00FF;
        hydraulic_states[HYDRAULIC_HA_NA][1] = pwm >> 8;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
      }
    break;
    case NID_Nebenarm:
      if (pwm == 0)
      {
        // set pwm value 0
        cmsg.id = 0x414;
        cmsg.len = 8;
        cmsg.data[4] = cmsg.data[5] = cmsg.data[6] = cmsg.data[7] = 0x00;
        for (uint8_t i = 4 ; i < 8 ; i++)
          hydraulic_states[HYDRAULIC_HA_NA][i] = 0;
        //memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
        
        // deactivate pwm
        cmsg.id = 0x314;
        cmsg.len = 2;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] & 0x30;
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8);    
        publishCanMessage(&cmsg);
        break;
      }
      if (ausleger[IDX_Nebenarm].state == MOVE_UP)
      {
        // activate Nebenarm up
        cmsg.id = 0x314;
        cmsg.len = 2;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] | 0x08; // pwm nebenarm ausfahren
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8);    
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        // set pwm value
        cmsg.id = 0x414;
        cmsg.len = 8;
        hydraulic_states[HYDRAULIC_HA_NA][4] = 0x00;
        hydraulic_states[HYDRAULIC_HA_NA][5] = 0x00;
        hydraulic_states[HYDRAULIC_HA_NA][6] = pwm & 0x00FF;
        hydraulic_states[HYDRAULIC_HA_NA][7] = pwm >> 8;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
      }
      else
      {
        // activate Nebenarm down
        cmsg.id = 0x314;
        cmsg.len = 2;
        hydraulic_states[HYDRAULIC_NA_VS][0] = hydraulic_states[HYDRAULIC_NA_VS][0] | 0x04; // pwm nebenarm einfahren
        hydraulic_states[HYDRAULIC_NA_VS][1] = 0x02;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_NA_VS], 8);  
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        // set pwm value
        cmsg.id = 0x414;
        cmsg.len = 8;
        hydraulic_states[HYDRAULIC_HA_NA][4] = pwm & 0x00FF;
        hydraulic_states[HYDRAULIC_HA_NA][5] = pwm >> 8;
        hydraulic_states[HYDRAULIC_HA_NA][6] = 0x00;
        hydraulic_states[HYDRAULIC_HA_NA][7] = 0x00;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_NA], 8);
        publishCanMessage(&cmsg);
      }
    break;
    case NID_Schnellwechselkopf:
      if (pwm == 0)
      {
        // set pwm value 0        
        cmsg.id = 0x494;
        cmsg.len = 8;
        memset(&cmsg.data,0x00,8);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
        
        // deactivate schnellwechselkopf
        cmsg.id = 0x214;
        cmsg.len = 3;
        hydraulic_states[HYDRAULIC_HA_SWK][0] = 0x00;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] &  0x03;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
        break;
      }
      if (ausleger[IDX_Schnellwechselkopf].state == MOVE_UP)
      {
        // activate schnellwechselkopf
        cmsg.id = 0x214;
        cmsg.len = 3;

        hydraulic_states[HYDRAULIC_HA_SWK][0] = 0x00;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] |  0x80;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        // set pwm value
        cmsg.id = 0x494;
        cmsg.len = 8;
        cmsg.data[4] = 0x00;
        cmsg.data[5] = 0x00;
        cmsg.data[6] = pwm & 0x00FF;
        cmsg.data[7] = pwm >> 8;
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
      }
      else
      {
        // activate schnellwechselkopf
        cmsg.id = 0x214;
        cmsg.len = 3;
        hydraulic_states[HYDRAULIC_HA_SWK][0] = 0x00;
        hydraulic_states[HYDRAULIC_HA_SWK][2] = hydraulic_states[HYDRAULIC_HA_SWK][2] |  0x40;
        memcpy(&cmsg.data,hydraulic_states[HYDRAULIC_HA_SWK], 3);
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);

        // set pwm value
        cmsg.id = 0x494;
        cmsg.len = 8;
        cmsg.data[4] = pwm & 0x00FF;
        cmsg.data[5] = pwm >> 8;
        cmsg.data[6] = 0x00;
        cmsg.data[7] = 0x00;
        publishCanMessage(&cmsg);
        memset(&cmsg.data,0x00,8);
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
  if (is_man_ctr)
    return 0;
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
        ROS_INFO_STREAM("New movement for node " << ausleger[i].node_name << " id:" << ausleger[i].node_id);
        // set pwm
        setPWM(ausleger[i].node_id, 1250);

        ausleger[i].is_moveing = true;
      }
    }
  }

  // check for stopping movement or changing
  for (uint8_t i = 0 ; i < 5 ; i++)
  {
    if (ausleger[i].is_moveing == true && ausleger[i].state >= MOVE_MOVING)
    {
      //ROS_INFO_STREAM("Check movement for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
      // check target
      if (ausleger[i].state == MOVE_THRUST_LEFT)
      {
        if (ausleger[i].position <= ausleger[i].target_position)
        {
          setPWM(ausleger[i].node_id,0); // stop moveing
          ROS_INFO_STREAM("Check movement l for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
          ausleger[i].is_moveing = false;
          ausleger[i].state = MOVE_HALT;
        }
      }
      if (ausleger[i].state == MOVE_THRUST_RIGHT)
      {
        if (ausleger[i].position >= ausleger[i].target_position)
        {
          setPWM(ausleger[i].node_id,0); // stop moveing
          ROS_INFO_STREAM("Check movement r for node " << ausleger[i].node_name << "::" << ausleger[i].position << "::" << ausleger[i].target_position);
          ausleger[i].is_moveing = false;
          ausleger[i].state = MOVE_HALT;
        }
      }
    }
  }

  // check for any active movement
  for (uint8_t i = 0; i < 5 ; i++)
  {
    if (ausleger[i].is_moveing == true)
      return 0;
  }
  
  if (Ausleger::pre_control_signal == PRE_CONTROL_SIG_OFF_ACK)
    return 0;
  
  // set pcs off and wait for ack
  cmdToCanMessage("pcs_off");
  Ausleger::pre_control_signal = PRE_CONTROL_SIG_OFF;
  // reset hydraulic states
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    memset(hydraulic_states[i], 0x00, 8);
  }
}

}
