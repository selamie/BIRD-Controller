#ifndef _ROS_SERVICE_TuneStrainGauge_h
#define _ROS_SERVICE_TuneStrainGauge_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

static const char TUNESTRAINGAUGE[] = "apc_arduino/TuneStrainGauge";

  class TuneStrainGaugeRequest : public ros::Msg
  {
    public:
      uint8_t sg_no;

    TuneStrainGaugeRequest():
      sg_no(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sg_no >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sg_no);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sg_no =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sg_no);
     return offset;
    }

    const char * getType(){ return TUNESTRAINGAUGE; };
    const char * getMD5(){ return "06c0a9a39d376f9edb73b66b05394056"; };

  };

  class TuneStrainGaugeResponse : public ros::Msg
  {
    public:

    TuneStrainGaugeResponse()
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

    const char * getType(){ return TUNESTRAINGAUGE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class TuneStrainGauge {
    public:
    typedef TuneStrainGaugeRequest Request;
    typedef TuneStrainGaugeResponse Response;
  };

}
#endif
