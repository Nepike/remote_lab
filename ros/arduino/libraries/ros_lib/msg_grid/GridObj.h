#ifndef _ROS_msg_grid_GridObj_h
#define _ROS_msg_grid_GridObj_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_grid
{

  class GridObj : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _clcenter_x_type;
      _clcenter_x_type clcenter_x;
      typedef int32_t _clcenter_y_type;
      _clcenter_y_type clcenter_y;
      typedef int32_t _clradius_type;
      _clradius_type clradius;
      typedef int32_t _clcolor_type;
      _clcolor_type clcolor;
      typedef int32_t _clskwer_type;
      _clskwer_type clskwer;

    GridObj():
      header(),
      clcenter_x(0),
      clcenter_y(0),
      clradius(0),
      clcolor(0),
      clskwer(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_clcenter_x;
      u_clcenter_x.real = this->clcenter_x;
      *(outbuffer + offset + 0) = (u_clcenter_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clcenter_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clcenter_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clcenter_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clcenter_x);
      union {
        int32_t real;
        uint32_t base;
      } u_clcenter_y;
      u_clcenter_y.real = this->clcenter_y;
      *(outbuffer + offset + 0) = (u_clcenter_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clcenter_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clcenter_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clcenter_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clcenter_y);
      union {
        int32_t real;
        uint32_t base;
      } u_clradius;
      u_clradius.real = this->clradius;
      *(outbuffer + offset + 0) = (u_clradius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clradius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clradius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clradius.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clradius);
      union {
        int32_t real;
        uint32_t base;
      } u_clcolor;
      u_clcolor.real = this->clcolor;
      *(outbuffer + offset + 0) = (u_clcolor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clcolor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clcolor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clcolor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clcolor);
      union {
        int32_t real;
        uint32_t base;
      } u_clskwer;
      u_clskwer.real = this->clskwer;
      *(outbuffer + offset + 0) = (u_clskwer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clskwer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clskwer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clskwer.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clskwer);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_clcenter_x;
      u_clcenter_x.base = 0;
      u_clcenter_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clcenter_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clcenter_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clcenter_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clcenter_x = u_clcenter_x.real;
      offset += sizeof(this->clcenter_x);
      union {
        int32_t real;
        uint32_t base;
      } u_clcenter_y;
      u_clcenter_y.base = 0;
      u_clcenter_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clcenter_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clcenter_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clcenter_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clcenter_y = u_clcenter_y.real;
      offset += sizeof(this->clcenter_y);
      union {
        int32_t real;
        uint32_t base;
      } u_clradius;
      u_clradius.base = 0;
      u_clradius.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clradius.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clradius.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clradius.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clradius = u_clradius.real;
      offset += sizeof(this->clradius);
      union {
        int32_t real;
        uint32_t base;
      } u_clcolor;
      u_clcolor.base = 0;
      u_clcolor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clcolor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clcolor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clcolor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clcolor = u_clcolor.real;
      offset += sizeof(this->clcolor);
      union {
        int32_t real;
        uint32_t base;
      } u_clskwer;
      u_clskwer.base = 0;
      u_clskwer.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clskwer.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clskwer.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clskwer.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clskwer = u_clskwer.real;
      offset += sizeof(this->clskwer);
     return offset;
    }

    virtual const char * getType() override { return "msg_grid/GridObj"; };
    virtual const char * getMD5() override { return "c703112996925bb66a43f708e2398ab5"; };

  };

}
#endif
