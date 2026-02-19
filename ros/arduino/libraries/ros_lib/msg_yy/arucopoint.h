#ifndef _ROS_msg_yy_arucopoint_h
#define _ROS_msg_yy_arucopoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_yy
{

  class arucopoint : public ros::Msg
  {
    public:
      typedef int16_t _id_type;
      _id_type id;
      typedef int16_t _x_type;
      _x_type x;
      typedef int16_t _y_type;
      _y_type y;
      typedef int16_t _weight_type;
      _weight_type weight;
      typedef int16_t _dist_type;
      _dist_type dist;

    arucopoint():
      id(0),
      x(0),
      y(0),
      weight(0),
      dist(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x);
      union {
        int16_t real;
        uint16_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y);
      union {
        int16_t real;
        uint16_t base;
      } u_weight;
      u_weight.real = this->weight;
      *(outbuffer + offset + 0) = (u_weight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weight.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->weight);
      union {
        int16_t real;
        uint16_t base;
      } u_dist;
      u_dist.real = this->dist;
      *(outbuffer + offset + 0) = (u_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dist.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dist);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        int16_t real;
        uint16_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        int16_t real;
        uint16_t base;
      } u_weight;
      u_weight.base = 0;
      u_weight.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_weight.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->weight = u_weight.real;
      offset += sizeof(this->weight);
      union {
        int16_t real;
        uint16_t base;
      } u_dist;
      u_dist.base = 0;
      u_dist.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dist.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dist = u_dist.real;
      offset += sizeof(this->dist);
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/arucopoint"; };
    virtual const char * getMD5() override { return "62a7cbf49d419fec43b64755d02b236d"; };

  };

}
#endif
