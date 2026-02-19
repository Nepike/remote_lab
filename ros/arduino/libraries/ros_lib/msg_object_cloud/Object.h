#ifndef _ROS_msg_object_cloud_Object_h
#define _ROS_msg_object_cloud_Object_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"

namespace msg_object_cloud
{

  class Object : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _type_type;
      _type_type type;
      typedef geometry_msgs::Pose2D _pose_type;
      _pose_type pose;
      enum { ROBOT = 1 };
      enum { OBSTACLE = 2 };

    Object():
      header(),
      type(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_type.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_type.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_type.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_type.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "msg_object_cloud/Object"; };
    virtual const char * getMD5() override { return "011b14416c674f1f26743ed62edf1835"; };

  };

}
#endif
