#ifndef _ROS_SERVICE_SuctionCupSwitch_h
#define _ROS_SERVICE_SuctionCupSwitch_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

static const char SUCTIONCUPSWITCH[] = "apc_arduino/SuctionCupSwitch";

  class SuctionCupSwitchRequest : public ros::Msg
  {
    public:
      uint8_t cup_no;
      bool suction;

    SuctionCupSwitchRequest():
      cup_no(0),
      suction(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cup_no >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cup_no);
      union {
        bool real;
        uint8_t base;
      } u_suction;
      u_suction.real = this->suction;
      *(outbuffer + offset + 0) = (u_suction.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->suction);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->cup_no =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cup_no);
      union {
        bool real;
        uint8_t base;
      } u_suction;
      u_suction.base = 0;
      u_suction.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->suction = u_suction.real;
      offset += sizeof(this->suction);
     return offset;
    }

    const char * getType(){ return SUCTIONCUPSWITCH; };
    const char * getMD5(){ return "1ae2a03ee71cb8a875ef556cf120e8e5"; };

  };

  class SuctionCupSwitchResponse : public ros::Msg
  {
    public:
      bool success;

    SuctionCupSwitchResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SUCTIONCUPSWITCH; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SuctionCupSwitch {
    public:
    typedef SuctionCupSwitchRequest Request;
    typedef SuctionCupSwitchResponse Response;
  };

}
#endif
