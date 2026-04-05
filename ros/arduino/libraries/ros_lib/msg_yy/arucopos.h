#ifndef _ROS_msg_yy_arucopos_h
#define _ROS_msg_yy_arucopos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "msg_yy/arucopoint.h"

namespace msg_yy
{

  class arucopos : public ros::Msg
  {
    public:
      typedef ros::Time _tm_type;
      _tm_type tm;
      uint32_t points_length;
      typedef msg_yy::arucopoint _points_type;
      _points_type st_points;
      _points_type * points;

    arucopos():
      tm(),
      points_length(0), st_points(), points(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->tm.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tm.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tm.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tm.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tm.sec);
      *(outbuffer + offset + 0) = (this->tm.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tm.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tm.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tm.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tm.nsec);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->tm.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->tm.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tm.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tm.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tm.sec);
      this->tm.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->tm.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tm.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tm.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tm.nsec);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (msg_yy::arucopoint*)realloc(this->points, points_lengthT * sizeof(msg_yy::arucopoint));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(msg_yy::arucopoint));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/arucopos"; };
    virtual const char * getMD5() override { return "e726434e3c404c8631fdc5f009776fd1"; };

  };

}
#endif
