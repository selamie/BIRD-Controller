#ifndef _ROS_SERVICE_ServiceBridge_h
#define _ROS_SERVICE_ServiceBridge_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

static const char SERVICEBRIDGE[] = "apc_arduino/ServiceBridge";

  class ServiceBridgeRequest : public ros::Msg
  {
    public:
      uint8_t service;
      uint8_t channel;
      int16_t value;

    ServiceBridgeRequest():
      service(0),
      channel(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->service >> (8 * 0)) & 0xFF;
      offset += sizeof(this->service);
      *(outbuffer + offset + 0) = (this->channel >> (8 * 0)) & 0xFF;
      offset += sizeof(this->channel);
      union {
        int16_t real;
        uint16_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->service =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->service);
      this->channel =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->channel);
      union {
        int16_t real;
        uint16_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return SERVICEBRIDGE; };
    const char * getMD5(){ return "b74f88db399aa5b0116a9e4b49d12af0"; };

  };

  class ServiceBridgeResponse : public ros::Msg
  {
    public:

    ServiceBridgeResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SERVICEBRIDGE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ServiceBridge {
    public:
    typedef ServiceBridgeRequest Request;
    typedef ServiceBridgeResponse Response;
  };

}
#endif
