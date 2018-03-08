#ifndef _ROS_apc_arduino_SuctionSensData_h
#define _ROS_apc_arduino_SuctionSensData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

  class SuctionSensData : public ros::Msg
  {
    public:
      uint16_t SucSen0;
      uint16_t SucSen1;

    SuctionSensData():
      SucSen0(0),
      SucSen1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->SucSen0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->SucSen0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->SucSen0);
      *(outbuffer + offset + 0) = (this->SucSen1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->SucSen1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->SucSen1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->SucSen0 =  ((uint16_t) (*(inbuffer + offset)));
      this->SucSen0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->SucSen0);
      this->SucSen1 =  ((uint16_t) (*(inbuffer + offset)));
      this->SucSen1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->SucSen1);
     return offset;
    }

    const char * getType(){ return "apc_arduino/SuctionSensData"; };
    const char * getMD5(){ return "ce09ba216647886aa9349ce4bcd73439"; };

  };

}
#endif