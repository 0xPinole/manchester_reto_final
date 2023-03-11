#ifndef _ROS_reto_final_motor_output_h
#define _ROS_reto_final_motor_output_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace reto_final
{

  class motor_output : public ros::Msg
  {
    public:
      typedef float _output_type;
      _output_type output;
      typedef float _tm_type;
      _tm_type tm;
      typedef const char* _st_type;
      _st_type st;

    motor_output():
      output(0),
      tm(0),
      st("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.real = this->output;
      *(outbuffer + offset + 0) = (u_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output);
      offset += serializeAvrFloat64(outbuffer + offset, this->tm);
      uint32_t length_st = strlen(this->st);
      varToArr(outbuffer + offset, length_st);
      offset += 4;
      memcpy(outbuffer + offset, this->st, length_st);
      offset += length_st;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.base = 0;
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output = u_output.real;
      offset += sizeof(this->output);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tm));
      uint32_t length_st;
      arrToVar(length_st, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st-1]=0;
      this->st = (char *)(inbuffer + offset-1);
      offset += length_st;
     return offset;
    }

    virtual const char * getType() override { return "reto_final/motor_output"; };
    virtual const char * getMD5() override { return "6607799b8130a5ce3bb33007e190e2cd"; };

  };

}
#endif
