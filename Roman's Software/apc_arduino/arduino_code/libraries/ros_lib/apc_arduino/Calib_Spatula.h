#ifndef _ROS_SERVICE_Calib_Spatula_h
#define _ROS_SERVICE_Calib_Spatula_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

static const char CALIB_SPATULA[] = "apc_arduino/Calib_Spatula";

  class Calib_SpatulaRequest : public ros::Msg
  {
    public:
      uint8_t sp_no;

    Calib_SpatulaRequest():
      sp_no(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sp_no >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sp_no);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sp_no =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sp_no);
     return offset;
    }

    const char * getType(){ return CALIB_SPATULA; };
    const char * getMD5(){ return "8cc92499159f551fa2ebea1960e4d2fb"; };

  };

  class Calib_SpatulaResponse : public ros::Msg
  {
    public:

    Calib_SpatulaResponse()
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

    const char * getType(){ return CALIB_SPATULA; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Calib_Spatula {
    public:
    typedef Calib_SpatulaRequest Request;
    typedef Calib_SpatulaResponse Response;
  };

}
#endif
