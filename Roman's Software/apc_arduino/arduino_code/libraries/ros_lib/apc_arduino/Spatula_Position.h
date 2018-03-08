#ifndef _ROS_apc_arduino_Spatula_Position_h
#define _ROS_apc_arduino_Spatula_Position_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

  class Spatula_Position : public ros::Msg
  {
    public:
      int32_t Sp1;
      int32_t Sp2;
      bool Sp1_moving;
      bool Sp2_moving;

    Spatula_Position():
      Sp1(0),
      Sp2(0),
      Sp1_moving(0),
      Sp2_moving(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Sp1;
      u_Sp1.real = this->Sp1;
      *(outbuffer + offset + 0) = (u_Sp1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Sp1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Sp1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Sp1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Sp1);
      union {
        int32_t real;
        uint32_t base;
      } u_Sp2;
      u_Sp2.real = this->Sp2;
      *(outbuffer + offset + 0) = (u_Sp2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Sp2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Sp2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Sp2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Sp2);
      union {
        bool real;
        uint8_t base;
      } u_Sp1_moving;
      u_Sp1_moving.real = this->Sp1_moving;
      *(outbuffer + offset + 0) = (u_Sp1_moving.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Sp1_moving);
      union {
        bool real;
        uint8_t base;
      } u_Sp2_moving;
      u_Sp2_moving.real = this->Sp2_moving;
      *(outbuffer + offset + 0) = (u_Sp2_moving.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Sp2_moving);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Sp1;
      u_Sp1.base = 0;
      u_Sp1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Sp1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Sp1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Sp1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Sp1 = u_Sp1.real;
      offset += sizeof(this->Sp1);
      union {
        int32_t real;
        uint32_t base;
      } u_Sp2;
      u_Sp2.base = 0;
      u_Sp2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Sp2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Sp2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Sp2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Sp2 = u_Sp2.real;
      offset += sizeof(this->Sp2);
      union {
        bool real;
        uint8_t base;
      } u_Sp1_moving;
      u_Sp1_moving.base = 0;
      u_Sp1_moving.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Sp1_moving = u_Sp1_moving.real;
      offset += sizeof(this->Sp1_moving);
      union {
        bool real;
        uint8_t base;
      } u_Sp2_moving;
      u_Sp2_moving.base = 0;
      u_Sp2_moving.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Sp2_moving = u_Sp2_moving.real;
      offset += sizeof(this->Sp2_moving);
     return offset;
    }

    const char * getType(){ return "apc_arduino/Spatula_Position"; };
    const char * getMD5(){ return "cdd45f2dabb9b9a191ebffff44325608"; };

  };

}
#endif