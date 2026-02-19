#ifndef _ROS_msg_robofob_irpult_h
#define _ROS_msg_robofob_irpult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_robofob
{

  class irpult : public ros::Msg
  {
    public:
      typedef uint8_t _addr_type;
      _addr_type addr;
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      typedef uint8_t _nump_type;
      _nump_type nump;
      typedef uint32_t _cnt_type;
      _cnt_type cnt;

    irpult():
      addr(0),
      cmd(0),
      nump(0),
      cnt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->addr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->addr);
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      *(outbuffer + offset + 0) = (this->nump >> (8 * 0)) & 0xFF;
      offset += sizeof(this->nump);
      *(outbuffer + offset + 0) = (this->cnt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cnt >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cnt >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cnt >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cnt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->addr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->addr);
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
      this->nump =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->nump);
      this->cnt =  ((uint32_t) (*(inbuffer + offset)));
      this->cnt |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cnt |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cnt |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cnt);
     return offset;
    }

    virtual const char * getType() override { return "msg_robofob/irpult"; };
    virtual const char * getMD5() override { return "1dc828461223436b40744513fa915335"; };

  };

}
#endif
