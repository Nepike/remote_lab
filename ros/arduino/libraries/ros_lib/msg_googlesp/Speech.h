#ifndef _ROS_msg_googlesp_Speech_h
#define _ROS_msg_googlesp_Speech_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_googlesp
{

  class Speech : public ros::Msg
  {
    public:
      typedef const char* _First_type;
      _First_type First;
      typedef const char* _Second_type;
      _Second_type Second;
      typedef float _Conf_type;
      _Conf_type Conf;

    Speech():
      First(""),
      Second(""),
      Conf(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_First = strlen(this->First);
      varToArr(outbuffer + offset, length_First);
      offset += 4;
      memcpy(outbuffer + offset, this->First, length_First);
      offset += length_First;
      uint32_t length_Second = strlen(this->Second);
      varToArr(outbuffer + offset, length_Second);
      offset += 4;
      memcpy(outbuffer + offset, this->Second, length_Second);
      offset += length_Second;
      union {
        float real;
        uint32_t base;
      } u_Conf;
      u_Conf.real = this->Conf;
      *(outbuffer + offset + 0) = (u_Conf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Conf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Conf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Conf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Conf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_First;
      arrToVar(length_First, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_First; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_First-1]=0;
      this->First = (char *)(inbuffer + offset-1);
      offset += length_First;
      uint32_t length_Second;
      arrToVar(length_Second, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Second; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Second-1]=0;
      this->Second = (char *)(inbuffer + offset-1);
      offset += length_Second;
      union {
        float real;
        uint32_t base;
      } u_Conf;
      u_Conf.base = 0;
      u_Conf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Conf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Conf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Conf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Conf = u_Conf.real;
      offset += sizeof(this->Conf);
     return offset;
    }

    virtual const char * getType() override { return "msg_googlesp/Speech"; };
    virtual const char * getMD5() override { return "58b9a6b9f0aeaeced812c5a0c3488d54"; };

  };

}
#endif
