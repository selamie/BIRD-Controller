#ifndef _ROS_apc_arduino_LEDData_h
#define _ROS_apc_arduino_LEDData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apc_arduino
{

  class LEDData : public ros::Msg
  {
    public:
      uint16_t LED0;
      uint16_t LED1;
      uint16_t LED2;
      uint16_t LED3;

    LEDData():
      LED0(0),
      LED1(0),
      LED2(0),
      LED3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->LED0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LED0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LED0);
      *(outbuffer + offset + 0) = (this->LED1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LED1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LED1);
      *(outbuffer + offset + 0) = (this->LED2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LED2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LED2);
      *(outbuffer + offset + 0) = (this->LED3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LED3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LED3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->LED0 =  ((uint16_t) (*(inbuffer + offset)));
      this->LED0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LED0);
      this->LED1 =  ((uint16_t) (*(inbuffer + offset)));
      this->LED1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LED1);
      this->LED2 =  ((uint16_t) (*(inbuffer + offset)));
      this->LED2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LED2);
      this->LED3 =  ((uint16_t) (*(inbuffer + offset)));
      this->LED3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LED3);
     return offset;
    }

    const char * getType(){ return "apc_arduino/LEDData"; };
    const char * getMD5(){ return "ba9ff9e389f7dfbf1058b1ca85db6aeb"; };

  };

}
#endif