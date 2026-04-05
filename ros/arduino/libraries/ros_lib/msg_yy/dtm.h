#ifndef _ROS_msg_yy_dtm_h
#define _ROS_msg_yy_dtm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_yy
{

  class dtm : public ros::Msg
  {
    public:
      typedef int16_t _cmd_type;
      _cmd_type cmd;
      typedef int16_t _arg1_type;
      _arg1_type arg1;
      typedef int16_t _arg2_type;
      _arg2_type arg2;
      typedef const char* _data_type;
      _data_type data;

    dtm():
      cmd(0),
      arg1(0),
      arg2(0),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cmd);
      union {
        int16_t real;
        uint16_t base;
      } u_arg1;
      u_arg1.real = this->arg1;
      *(outbuffer + offset + 0) = (u_arg1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arg1);
      union {
        int16_t real;
        uint16_t base;
      } u_arg2;
      u_arg2.real = this->arg2;
      *(outbuffer + offset + 0) = (u_arg2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arg2);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
      union {
        int16_t real;
        uint16_t base;
      } u_arg1;
      u_arg1.base = 0;
      u_arg1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arg1 = u_arg1.real;
      offset += sizeof(this->arg1);
      union {
        int16_t real;
        uint16_t base;
      } u_arg2;
      u_arg2.base = 0;
      u_arg2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arg2 = u_arg2.real;
      offset += sizeof(this->arg2);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/dtm"; };
    virtual const char * getMD5() override { return "95d2487c05ffaeeb265873cc5ead084b"; };

  };

}
#endif
