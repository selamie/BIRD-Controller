#ifndef _ROS_apc_arduino_StrainGaugeData_h
#define _ROS_apc_arduino_StrainGaugeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

  class StrainGaugeData : public ros::Msg
  {
    public:
      int16_t strain0;
      uint8_t level0;
      int16_t strain1;
      uint8_t level1;
      int16_t strain2;
      uint8_t level2;
      int16_t strain3;
      uint8_t level3;

    StrainGaugeData():
      strain0(0),
      level0(0),
      strain1(0),
      level1(0),
      strain2(0),
      level2(0),
      strain3(0),
      level3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_strain0;
      u_strain0.real = this->strain0;
      *(outbuffer + offset + 0) = (u_strain0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_strain0.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->strain0);
      *(outbuffer + offset + 0) = (this->level0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level0);
      union {
        int16_t real;
        uint16_t base;
      } u_strain1;
      u_strain1.real = this->strain1;
      *(outbuffer + offset + 0) = (u_strain1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_strain1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->strain1);
      *(outbuffer + offset + 0) = (this->level1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level1);
      union {
        int16_t real;
        uint16_t base;
      } u_strain2;
      u_strain2.real = this->strain2;
      *(outbuffer + offset + 0) = (u_strain2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_strain2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->strain2);
      *(outbuffer + offset + 0) = (this->level2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level2);
      union {
        int16_t real;
        uint16_t base;
      } u_strain3;
      u_strain3.real = this->strain3;
      *(outbuffer + offset + 0) = (u_strain3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_strain3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->strain3);
      *(outbuffer + offset + 0) = (this->level3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_strain0;
      u_strain0.base = 0;
      u_strain0.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_strain0.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strain0 = u_strain0.real;
      offset += sizeof(this->strain0);
      this->level0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level0);
      union {
        int16_t real;
        uint16_t base;
      } u_strain1;
      u_strain1.base = 0;
      u_strain1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_strain1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strain1 = u_strain1.real;
      offset += sizeof(this->strain1);
      this->level1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level1);
      union {
        int16_t real;
        uint16_t base;
      } u_strain2;
      u_strain2.base = 0;
      u_strain2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_strain2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strain2 = u_strain2.real;
      offset += sizeof(this->strain2);
      this->level2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level2);
      union {
        int16_t real;
        uint16_t base;
      } u_strain3;
      u_strain3.base = 0;
      u_strain3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_strain3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strain3 = u_strain3.real;
      offset += sizeof(this->strain3);
      this->level3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->level3);
     return offset;
    }

    const char * getType(){ return "apc_arduino/StrainGaugeData"; };
    const char * getMD5(){ return "c2a6f59ffa4a2798ea4a36fc6f94a382"; };

  };

}
#endif