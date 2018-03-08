#ifndef _ROS_SERVICE_MoveToPos_h
#define _ROS_SERVICE_MoveToPos_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

static const char MOVETOPOS[] = "apc_arduino/MoveToPos";

  class MoveToPosRequest : public ros::Msg
  {
    public:
      uint8_t sp_no;
      int16_t pos;

    MoveToPosRequest():
      sp_no(0),
      pos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sp_no >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sp_no);
      union {
        int16_t real;
        uint16_t base;
      } u_pos;
      u_pos.real = this->pos;
      *(outbuffer + offset + 0) = (u_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sp_no =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sp_no);
      union {
        int16_t real;
        uint16_t base;
      } u_pos;
      u_pos.base = 0;
      u_pos.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pos = u_pos.real;
      offset += sizeof(this->pos);
     return offset;
    }

    const char * getType(){ return MOVETOPOS; };
    const char * getMD5(){ return "80bf00435ef02088350aec13d2fc15aa"; };

  };

  class MoveToPosResponse : public ros::Msg
  {
    public:

    MoveToPosResponse()
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

    const char * getType(){ return MOVETOPOS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class MoveToPos {
    public:
    typedef MoveToPosRequest Request;
    typedef MoveToPosResponse Response;
  };

}
#endif
