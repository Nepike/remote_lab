#ifndef _ROS_msg_kvorum2_sensors_h
#define _ROS_msg_kvorum2_sensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "msg_kvorum2/sdata.h"

namespace msg_kvorum2
{

  class sensors : public ros::Msg
  {
    public:
      typedef int32_t _src_type;
      _src_type src;
      typedef int32_t _dest_type;
      _dest_type dest;
      typedef int32_t _tm_type;
      _tm_type tm;
      uint32_t data_length;
      typedef msg_kvorum2::sdata _data_type;
      _data_type st_data;
      _data_type * data;

    sensors():
      src(0),
      dest(0),
      tm(0),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_src;
      u_src.real = this->src;
      *(outbuffer + offset + 0) = (u_src.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_src.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_src.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_src.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->src);
      union {
        int32_t real;
        uint32_t base;
      } u_dest;
      u_dest.real = this->dest;
      *(outbuffer + offset + 0) = (u_dest.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dest.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dest.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dest.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dest);
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
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_src;
      u_src.base = 0;
      u_src.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_src.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_src.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_src.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->src = u_src.real;
      offset += sizeof(this->src);
      union {
        int32_t real;
        uint32_t base;
      } u_dest;
      u_dest.base = 0;
      u_dest.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dest.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dest.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dest.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dest = u_dest.real;
      offset += sizeof(this->dest);
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
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (msg_kvorum2::sdata*)realloc(this->data, data_lengthT * sizeof(msg_kvorum2::sdata));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(msg_kvorum2::sdata));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/sensors"; };
    virtual const char * getMD5() override { return "3241db82928f1d22e38d04240df7677d"; };

  };

}
#endif
