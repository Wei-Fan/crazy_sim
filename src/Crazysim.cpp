//#include <regex>
#include <mutex>

#include "Crazysim.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

std::mutex g_mutex[MAX_RADIOS];

Crazyflie::Crazyflie(
  const std::string& link_uri)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
  , m_logInfo()
  , m_logTocEntries()
  , m_logTocEntriesRequested()
  , m_logBlockCb()
  , m_blockReset(false)
  , m_blockCreated()
  , m_blockStarted()
  , m_blockStopped()
  , m_paramInfo()
  , m_paramTocEntries()
  , m_paramTocEntriesRequested()
  , m_paramValues()
  , m_paramValuesRequested()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%lx",
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    {
      std::unique_lock<std::mutex> mlock(g_mutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void Crazyflie::logReset()
{
  m_blockReset = false;
  do {
    crtpLogResetRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (!m_blockReset);
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest request(roll, pitch, yawrate, thrust);
  sendPacket((const uint8_t*)&request, sizeof(request));
}

void Crazyflie::sendPing()
{
  uint8_t ping = 0xFF;
  sendPacket(&ping, sizeof(ping));
}

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_firmware[] = {0xFF, 0xFE, 0xF0, 0x01};
  while(!sendPacket(reboot_to_firmware, sizeof(reboot_to_firmware))) {}
}

void Crazyflie::rebootToBootloader()
{
  const uint8_t reboot_init[] = {0xFF, 0xFE, 0xFF};
  while(!sendPacket(reboot_init, sizeof(reboot_init))) {}

  const uint8_t reboot_to_bootloader[] = {0xFF, 0xFE, 0xF0, 0x00};
  while(!sendPacket(reboot_to_bootloader, sizeof(reboot_to_bootloader))) {}
}

void Crazyflie::requestLogToc()
{

  // Find the number of log variables in TOC
  m_logInfo.len = 0;
  do
  {
    crtpLogGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_logInfo.len == 0);
  std::cout << "Log: " << (int)m_logInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_logTocEntriesRequested.clear();
  m_logTocEntries.resize(m_logInfo.len);
  for (size_t i = 0; i < m_logInfo.len; ++i)
  {
    m_logTocEntriesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_logTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_logTocEntriesRequested.size(); ++p)
    {
      auto iter = m_logTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpLogGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::requestParamToc()
{
  // Find the number of log variables in TOC
  m_paramInfo.len = 0;
  do
  {
    crtpParamTocGetInfoRequest request;
    sendPacket((const uint8_t*)&request, sizeof(request));
  } while(m_paramInfo.len == 0);
  std::cout << "Params: " << (int)m_paramInfo.len << std::endl;

  // Prepare data structures to request detailed information
  m_paramTocEntriesRequested.clear();
  m_paramValues.clear();
  m_paramTocEntries.resize(m_paramInfo.len);
  for (size_t i = 0; i < m_paramInfo.len; ++i)
  {
    m_paramTocEntriesRequested.insert(i);
    m_paramValuesRequested.insert(i);
  }

  // Request detailed information, until done
  while (m_paramTocEntriesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramTocEntriesRequested.size(); ++p)
    {
      auto iter = m_paramTocEntriesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamTocGetItemRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }

  // Request current values
  while (m_paramValuesRequested.size() > 0)
  {
    for (size_t p = 0; p < m_paramValuesRequested.size(); ++p)
    {
      auto iter = m_paramValuesRequested.begin();
      for (size_t j = 0; j < p; ++j) {
        ++iter;
      }
      size_t i = *iter;
      crtpParamReadRequest request(i);
      sendPacket((const uint8_t*)&request, sizeof(request));
    }
  }
}

void Crazyflie::setParam(uint8_t id, const ParamValue& value) {

  m_paramValues.erase(id);
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      do
      {
        switch (entry.type) {
          case ParamTypeUint8:
            {
              crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt8:
            {
              crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint16:
            {
              crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt16:
            {
              crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeUint32:
            {
              crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeInt32:
            {
              crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
          case ParamTypeFloat:
            {
              crtpParamWriteRequest<float> request(id, value.valueFloat);
              sendPacket((const uint8_t*)&request, sizeof(request));
              break;
            }
        }
      } while(m_paramValues.find(id) == m_paramValues.end());
      break;
    }
  }
}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
}


