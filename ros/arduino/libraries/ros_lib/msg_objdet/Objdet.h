#ifndef _ROS_msg_objdet_Objdet_h
#define _ROS_msg_objdet_Objdet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_objdet
{

  class Objdet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _id_type;
      _id_type id;
      typedef int32_t _x_type;
      _x_type x;
      typedef int32_t _y_type;
      _y_type y;
      typedef int32_t _w_type;
      _w_type w;
      typedef int32_t _h_type;
      _h_type h;
      typedef int32_t _track_type;
      _track_type track;
      typedef float _conf_coeff_type;
      _conf_coeff_type conf_coeff;
      typedef float _distance_type;
      _distance_type distance;

    Objdet():
      header(),
      id(0),
      x(0),
      y(0),
      w(0),
      h(0),
      track(0),
      conf_coeff(0),
      distance(0)
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
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        int32_t real;
        uint32_t base;
      } u_w;
      u_w.real = this->w;
      *(outbuffer + offset + 0) = (u_w.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w);
      union {
        int32_t real;
        uint32_t base;
      } u_h;
      u_h.real = this->h;
      *(outbuffer + offset + 0) = (u_h.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_h.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_h.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_h.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->h);
      union {
        int32_t real;
        uint32_t base;
      } u_track;
      u_track.real = this->track;
      *(outbuffer + offset + 0) = (u_track.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_track.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_track.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_track.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->track);
      offset += serializeAvrFloat64(outbuffer + offset, this->conf_coeff);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
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
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        int32_t real;
        uint32_t base;
      } u_w;
      u_w.base = 0;
      u_w.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w = u_w.real;
      offset += sizeof(this->w);
      union {
        int32_t real;
        uint32_t base;
      } u_h;
      u_h.base = 0;
      u_h.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_h.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_h.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_h.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->h = u_h.real;
      offset += sizeof(this->h);
      union {
        int32_t real;
        uint32_t base;
      } u_track;
      u_track.base = 0;
      u_track.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_track.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_track.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_track.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->track = u_track.real;
      offset += sizeof(this->track);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->conf_coeff));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
     return offset;
    }

    virtual const char * getType() override { return "msg_objdet/Objdet"; };
    virtual const char * getMD5() override { return "7648ed6a0040d552d796ed05fff4a6de"; };

  };

}
#endif
