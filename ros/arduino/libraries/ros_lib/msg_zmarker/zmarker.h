#ifndef _ROS_msg_zmarker_zmarker_h
#define _ROS_msg_zmarker_zmarker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_zmarker
{

  class zmarker : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _data_type;
      _data_type data;
      typedef int32_t _center_x_type;
      _center_x_type center_x;
      typedef int32_t _center_y_type;
      _center_y_type center_y;
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;

    zmarker():
      header(),
      data(""),
      center_x(0),
      center_y(0),
      width(0),
      height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_center_x;
      u_center_x.real = this->center_x;
      *(outbuffer + offset + 0) = (u_center_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_center_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_center_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_center_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->center_x);
      union {
        int32_t real;
        uint32_t base;
      } u_center_y;
      u_center_y.real = this->center_y;
      *(outbuffer + offset + 0) = (u_center_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_center_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_center_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_center_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->center_y);
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      union {
        int32_t real;
        uint32_t base;
      } u_center_x;
      u_center_x.base = 0;
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_center_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->center_x = u_center_x.real;
      offset += sizeof(this->center_x);
      union {
        int32_t real;
        uint32_t base;
      } u_center_y;
      u_center_y.base = 0;
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_center_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->center_y = u_center_y.real;
      offset += sizeof(this->center_y);
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
     return offset;
    }

    virtual const char * getType() override { return "msg_zmarker/zmarker"; };
    virtual const char * getMD5() override { return "f766cadf6251cd4ef1a67c21ce5a29cd"; };

  };

}
#endif
