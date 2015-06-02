#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdio>
#include <stdint.h>
#include "ros/ros.h"
#include "can_core.h"
#include "tinycan/CanMsg.h"
#include "can_drv.h"
#include "config.h"
#include "global.h"

namespace tinycan {

/*----------------------------------------------------------------------
 * Can()
 * Constructor.
 *--------------------------------------------------------------------*/

Can::Can()
{
  device_idx = 0;
  return;
}

/*----------------------------------------------------------------------
 * ~Can()
 * Destructor.
 *--------------------------------------------------------------------*/

Can::~Can()
{
  CanDownDriver();
  // **** DLL entladen
  UnloadDriver();
}

/*----------------------------------------------------------------------
 * init()
 * Init tinycan device.
 *--------------------------------------------------------------------*/

int Can::init()
{
  int err;

  if ((err = LoadDriver(TREIBER_NAME)) < 0)
  {
    std::cout << "error! code " << err <<std::endl;
    return false;
  }

  if ((err = CanInitDriver((char*)TREIBER_INIT)) != 0)
  {
    std::cout << "error! code " << err <<std::endl;
    return false;
  }

  if ((err = CanDeviceOpen(device_idx, DEVICE_OPEN)) != 0)
  {
    std::cout << "error! code " << err <<std::endl;
    return false;
  }

  /******************************/
  /*  CAN Speed einstellen      */
  /******************************/
  // **** Übertragungsgeschwindigkeit auf 250kBit/s einstellen
  CanSetSpeed(device_idx, CAN_SPEED);

  // **** CAN Bus Start, alle FIFOs, Filter, Puffer und Fehler löschen
  CanSetMode(device_idx, OP_CAN_START, CAN_CMD_ALL_CLEAR);

  return true;
}

/*----------------------------------------------------------------------
 * buildMsg()
 * Build a can message from given data.
 *--------------------------------------------------------------------*/

int Can::buildMsg(TCanMsg *msg, uint8_t id, uint8_t len, char* data)
{
  // msg Variable Initialisieren
  msg->MsgFlags = 0L; // Alle Flags löschen, Stanadrt Frame Format,
                      // keine RTR, Datenlänge auf 0

  //msg->MsgRTR = 1;  // Nachricht als RTR Frame versenden
  //msg->MsgEFF = 1;  // Nachricht im EFF (Ext. Frame Format) versenden

  msg->Id = id;
  msg->MsgLen = len;
  memcpy(msg->MsgData, data, len);
  return true;
}

/*----------------------------------------------------------------------
 * sendMsg()
 * Send a message via tinycan device.
 *--------------------------------------------------------------------*/

int Can::sendMsg(TCanMsg *msg)
{
  int err = 0;

  if ((err = CanTransmit(device_idx, msg, 1)) < 0)
  {
    std::cout << "CanTransmit Error-Code:" << err << std::endl;
    return false;
  }
  return true;
}

/*----------------------------------------------------------------------
 * readMsg()
 * Read a can message from tinycan device.
 *--------------------------------------------------------------------*/

int Can::readMsg(TCanMsg *msg)
{
  struct TDeviceStatus status;

  CanGetDeviceStatus(device_idx, &status);

  if (status.DrvStatus >= DRV_STATUS_CAN_OPEN) 
  {
    if (status.CanStatus == CAN_STATUS_BUS_OFF)
    {
      std::cout << "CAN Status BusOff" << std::endl;
      CanSetMode(0, OP_CAN_RESET, CAN_CMD_NONE);
    }
  }
  else 
  {
    std::cout << status.DrvStatus << std::endl;
    std::cout << "CAN Device nicht geöffnet" << std::endl;
    return false;
  }

  if (CanReceive(device_idx, msg, 1) > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
  return true;
}

/*----------------------------------------------------------------------
 * printMsg()
 * Print a can message.
 *--------------------------------------------------------------------*/

int Can::printMsg(TCanMsg *msg) 
{
  if (msg == NULL)
    return false;
  //std::cout <<  msg->Time.Sec << "." << msg->Time.USec << std::endl;
  std::cout << "id:" << msg->Id << " dlc:" << msg->MsgLen << std::endl << "data:" << std::endl;
  if (msg->MsgLen)
  {
    for (int i = 0; i < msg->MsgLen; i++)
    {
      if (i !=0)
        std::cout << "::";
      printf("%.2X", msg->MsgData[i]);
    }
  }
  else
    std::cout << " keine";
  std::cout << std::endl;
  return true;
}

/*----------------------------------------------------------------------
 * callbackCanMessage()
 * Callback function for can subscriber.
 *--------------------------------------------------------------------*/

void Can::callbackCanMessage(const tinycan::CanMsg::ConstPtr& msg)
{
  if (msg == NULL)
    return;
  TCanMsg tmsg;
  tmsg.Id = msg->id;
  tmsg.MsgLen = msg->len;
  for (int i = 0 ; i < msg->len ; i++)
    tmsg.MsgData[i] = msg->data[i];
  sendMsg(&tmsg);
}

/*----------------------------------------------------------------------
 * publishCanMessage()
 * Publish can message.
 *--------------------------------------------------------------------*/

void Can::publishCanMessage(ros::Publisher *pub_message, TCanMsg *tmsg)
{
  ROS_INFO_STREAM("pub can message");
  if (tmsg == NULL)
    return;
  tinycan::CanMsg msg;
  msg.id = tmsg->Id;
  msg.len = tmsg->MsgLen;
  for (int i = 0 ; i < msg.len ; i++)
    msg.data[i] = tmsg->MsgData[i];
  pub_message->publish(msg);
}
}
