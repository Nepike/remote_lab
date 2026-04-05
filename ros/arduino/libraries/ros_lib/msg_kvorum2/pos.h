#ifndef _ROS_msg_kvorum2_pos_h
#define _ROS_msg_kvorum2_pos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_kvorum2
{

  class pos : public ros::Msg
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
      typedef int32_t _a_type;
      _a_type a;
      typedef float _cs_type;
      _cs_type cs;
      typedef const char* _shape_type;
      _shape_type shape;
      typedef bool _traceOn_type;
      _traceOn_type traceOn;
      typedef int32_t _state_type;
      _state_type state;
      typedef bool _show_id_type;
      _show_id_type show_id;
      typedef bool _alive_type;
      _alive_type alive;

    pos():
      header(),
      id(0),
      x(0),
      y(0),
      a(0),
      cs(0),
      shape(""),
      traceOn(0),
      state(0),
      show_id(0),
      alive(0)
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
      } u_a;
      u_a.real = this->a;
      *(outbuffer + offset + 0) = (u_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a);
      union {
        float real;
        uint32_t base;
      } u_cs;
      u_cs.real = this->cs;
      *(outbuffer + offset + 0) = (u_cs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cs);
      uint32_t length_shape = strlen(this->shape);
      varToArr(outbuffer + offset, length_shape);
      offset += 4;
      memcpy(outbuffer + offset, this->shape, length_shape);
      offset += length_shape;
      union {
        bool real;
        uint8_t base;
      } u_traceOn;
      u_traceOn.real = this->traceOn;
      *(outbuffer + offset + 0) = (u_traceOn.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->traceOn);
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_show_id;
      u_show_id.real = this->show_id;
      *(outbuffer + offset + 0) = (u_show_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->show_id);
      union {
        bool real;
        uint8_t base;
      } u_alive;
      u_alive.real = this->alive;
      *(outbuffer + offset + 0) = (u_alive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->alive);
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
      } u_a;
      u_a.base = 0;
      u_a.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a = u_a.real;
      offset += sizeof(this->a);
      union {
        float real;
        uint32_t base;
      } u_cs;
      u_cs.base = 0;
      u_cs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cs = u_cs.real;
      offset += sizeof(this->cs);
      uint32_t length_shape;
      arrToVar(length_shape, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_shape; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_shape-1]=0;
      this->shape = (char *)(inbuffer + offset-1);
      offset += length_shape;
      union {
        bool real;
        uint8_t base;
      } u_traceOn;
      u_traceOn.base = 0;
      u_traceOn.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->traceOn = u_traceOn.real;
      offset += sizeof(this->traceOn);
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_show_id;
      u_show_id.base = 0;
      u_show_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->show_id = u_show_id.real;
      offset += sizeof(this->show_id);
      union {
        bool real;
        uint8_t base;
      } u_alive;
      u_alive.base = 0;
      u_alive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->alive = u_alive.real;
      offset += sizeof(this->alive);
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/pos"; };
    virtual const char * getMD5() override { return "abf29e2960f6dad9d54a776fb36f0cf0"; };

  };

}
#endif
