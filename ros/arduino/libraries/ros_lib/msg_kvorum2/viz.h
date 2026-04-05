#ifndef _ROS_msg_kvorum2_viz_h
#define _ROS_msg_kvorum2_viz_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "msg_kvorum2/pos.h"

namespace msg_kvorum2
{

  class viz : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _cmd_type;
      _cmd_type cmd;
      uint32_t data_length;
      typedef msg_kvorum2::pos _data_type;
      _data_type st_data;
      _data_type * data;
      typedef int32_t _tm_type;
      _tm_type tm;

    viz():
      header(),
      cmd(0),
      data_length(0), st_data(), data(nullptr),
      tm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_tm;
      u_tm.real = this->tm;
      *(outbuffer + offset + 0) = (u_tm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (msg_kvorum2::pos*)realloc(this->data, data_lengthT * sizeof(msg_kvorum2::pos));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(msg_kvorum2::pos));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_tm;
      u_tm.base = 0;
      u_tm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tm = u_tm.real;
      offset += sizeof(this->tm);
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/viz"; };
    virtual const char * getMD5() override { return "69a3d1661d841890b61dc294dd185087"; };

  };

}
#endif
