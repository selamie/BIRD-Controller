#ifndef _ROS_apc_arduino_HallEffectData_h
#define _ROS_apc_arduino_HallEffectData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

  class HallEffectData : public ros::Msg
  {
    public:
      uint16_t HE0;
      uint16_t HE1;
      uint16_t HE2;
      uint16_t HE3;

    HallEffectData():
      HE0(0),
      HE1(0),
      HE2(0),
      HE3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->HE0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->HE0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->HE0);
      *(outbuffer + offset + 0) = (this->HE1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->HE1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->HE1);
      *(outbuffer + offset + 0) = (this->HE2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->HE2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->HE2);
      *(outbuffer + offset + 0) = (this->HE3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->HE3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->HE3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->HE0 =  ((uint16_t) (*(inbuffer + offset)));
      this->HE0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->HE0);
      this->HE1 =  ((uint16_t) (*(inbuffer + offset)));
      this->HE1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->HE1);
      this->HE2 =  ((uint16_t) (*(inbuffer + offset)));
      this->HE2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->HE2);
      this->HE3 =  ((uint16_t) (*(inbuffer + offset)));
      this->HE3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->HE3);
     return offset;
    }

    const char * getType(){ return "apc_arduino/HallEffectData"; };
    const char * getMD5(){ return "45bf37d1a93e6cd163dd5d8f03134e83"; };

  };

}
#endif