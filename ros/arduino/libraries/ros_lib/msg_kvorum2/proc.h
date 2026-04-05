#ifndef _ROS_msg_kvorum2_proc_h
#define _ROS_msg_kvorum2_proc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_kvorum2
{

  class proc : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _id_type;
      _id_type id;
      typedef int32_t _proc_type;
      _proc_type proc;
      typedef int32_t _arg1_type;
      _arg1_type arg1;
      typedef int32_t _arg2_type;
      _arg2_type arg2;
      typedef int32_t _arg3_type;
      _arg3_type arg3;
      typedef int32_t _arg4_type;
      _arg4_type arg4;

    proc():
      header(),
      id(0),
      proc(0),
      arg1(0),
      arg2(0),
      arg3(0),
      arg4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_proc;
      u_proc.real = this->proc;
      *(outbuffer + offset + 0) = (u_proc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_proc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_proc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_proc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->proc);
      union {
        int32_t real;
        uint32_t base;
      } u_arg1;
      u_arg1.real = this->arg1;
      *(outbuffer + offset + 0) = (u_arg1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg1);
      union {
        int32_t real;
        uint32_t base;
      } u_arg2;
      u_arg2.real = this->arg2;
      *(outbuffer + offset + 0) = (u_arg2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg2);
      union {
        int32_t real;
        uint32_t base;
      } u_arg3;
      u_arg3.real = this->arg3;
      *(outbuffer + offset + 0) = (u_arg3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg3);
      union {
        int32_t real;
        uint32_t base;
      } u_arg4;
      u_arg4.real = this->arg4;
      *(outbuffer + offset + 0) = (u_arg4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_proc;
      u_proc.base = 0;
      u_proc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_proc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_proc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_proc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->proc = u_proc.real;
      offset += sizeof(this->proc);
      union {
        int32_t real;
        uint32_t base;
      } u_arg1;
      u_arg1.base = 0;
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg1 = u_arg1.real;
      offset += sizeof(this->arg1);
      union {
        int32_t real;
        uint32_t base;
      } u_arg2;
      u_arg2.base = 0;
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg2 = u_arg2.real;
      offset += sizeof(this->arg2);
      union {
        int32_t real;
        uint32_t base;
      } u_arg3;
      u_arg3.base = 0;
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg3 = u_arg3.real;
      offset += sizeof(this->arg3);
      union {
        int32_t real;
        uint32_t base;
      } u_arg4;
      u_arg4.base = 0;
      u_arg4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg4 = u_arg4.real;
      offset += sizeof(this->arg4);
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/proc"; };
    virtual const char * getMD5() override { return "c6983dea4429c11215fc4935d2abce41"; };

  };

}
#endif
