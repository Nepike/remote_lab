#ifndef _ROS_msg_spot_Spot_h
#define _ROS_msg_spot_Spot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_spot
{

  class Spot : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _point0_x_type;
      _point0_x_type point0_x;
      typedef int32_t _point0_y_type;
      _point0_y_type point0_y;
      typedef int32_t _point1_x_type;
      _point1_x_type point1_x;
      typedef int32_t _point1_y_type;
      _point1_y_type point1_y;
      typedef int32_t _point2_x_type;
      _point2_x_type point2_x;
      typedef int32_t _point2_y_type;
      _point2_y_type point2_y;
      typedef int32_t _point3_x_type;
      _point3_x_type point3_x;
      typedef int32_t _point3_y_type;
      _point3_y_type point3_y;
      typedef int32_t _center_x_type;
      _center_x_type center_x;
      typedef int32_t _center_y_type;
      _center_y_type center_y;
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;
      typedef int32_t _radius_type;
      _radius_type radius;
      typedef int32_t _color_type;
      _color_type color;
      typedef float _angle_type;
      _angle_type angle;

    Spot():
      header(),
      point0_x(0),
      point0_y(0),
      point1_x(0),
      point1_y(0),
      point2_x(0),
      point2_y(0),
      point3_x(0),
      point3_y(0),
      center_x(0),
      center_y(0),
      width(0),
      height(0),
      radius(0),
      color(0),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_point0_x;
      u_point0_x.real = this->point0_x;
      *(outbuffer + offset + 0) = (u_point0_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point0_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point0_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point0_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point0_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point0_y;
      u_point0_y.real = this->point0_y;
      *(outbuffer + offset + 0) = (u_point0_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point0_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point0_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point0_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point0_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point1_x;
      u_point1_x.real = this->point1_x;
      *(outbuffer + offset + 0) = (u_point1_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point1_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point1_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point1_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point1_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point1_y;
      u_point1_y.real = this->point1_y;
      *(outbuffer + offset + 0) = (u_point1_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point1_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point1_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point1_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point1_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point2_x;
      u_point2_x.real = this->point2_x;
      *(outbuffer + offset + 0) = (u_point2_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point2_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point2_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point2_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point2_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point2_y;
      u_point2_y.real = this->point2_y;
      *(outbuffer + offset + 0) = (u_point2_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point2_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point2_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point2_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point2_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point3_x;
      u_point3_x.real = this->point3_x;
      *(outbuffer + offset + 0) = (u_point3_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point3_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point3_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point3_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point3_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point3_y;
      u_point3_y.real = this->point3_y;
      *(outbuffer + offset + 0) = (u_point3_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_point3_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_point3_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_point3_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->point3_y);
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
      union {
        int32_t real;
        uint32_t base;
      } u_radius;
      u_radius.real = this->radius;
      *(outbuffer + offset + 0) = (u_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->radius);
      union {
        int32_t real;
        uint32_t base;
      } u_color;
      u_color.real = this->color;
      *(outbuffer + offset + 0) = (u_color.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_color.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_color.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_color.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_point0_x;
      u_point0_x.base = 0;
      u_point0_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point0_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point0_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point0_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point0_x = u_point0_x.real;
      offset += sizeof(this->point0_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point0_y;
      u_point0_y.base = 0;
      u_point0_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point0_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point0_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point0_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point0_y = u_point0_y.real;
      offset += sizeof(this->point0_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point1_x;
      u_point1_x.base = 0;
      u_point1_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point1_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point1_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point1_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point1_x = u_point1_x.real;
      offset += sizeof(this->point1_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point1_y;
      u_point1_y.base = 0;
      u_point1_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point1_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point1_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point1_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point1_y = u_point1_y.real;
      offset += sizeof(this->point1_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point2_x;
      u_point2_x.base = 0;
      u_point2_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point2_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point2_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point2_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point2_x = u_point2_x.real;
      offset += sizeof(this->point2_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point2_y;
      u_point2_y.base = 0;
      u_point2_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point2_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point2_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point2_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point2_y = u_point2_y.real;
      offset += sizeof(this->point2_y);
      union {
        int32_t real;
        uint32_t base;
      } u_point3_x;
      u_point3_x.base = 0;
      u_point3_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point3_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point3_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point3_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point3_x = u_point3_x.real;
      offset += sizeof(this->point3_x);
      union {
        int32_t real;
        uint32_t base;
      } u_point3_y;
      u_point3_y.base = 0;
      u_point3_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_point3_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_point3_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_point3_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->point3_y = u_point3_y.real;
      offset += sizeof(this->point3_y);
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
      union {
        int32_t real;
        uint32_t base;
      } u_radius;
      u_radius.base = 0;
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->radius = u_radius.real;
      offset += sizeof(this->radius);
      union {
        int32_t real;
        uint32_t base;
      } u_color;
      u_color.base = 0;
      u_color.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_color.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_color.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_color.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->color = u_color.real;
      offset += sizeof(this->color);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
     return offset;
    }

    virtual const char * getType() override { return "msg_spot/Spot"; };
    virtual const char * getMD5() override { return "de21cec50f9b1499dcebbf181e8c13e1"; };

  };

}
#endif
