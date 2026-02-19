#ifndef _ROS_msg_crtcoord_crtcoord_h
#define _ROS_msg_crtcoord_crtcoord_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Odometry.h"

namespace msg_crtcoord
{

  class crtcoord : public ros::Msg
  {
    public:
      typedef nav_msgs::Odometry _pos_type;
      _pos_type pos;

    crtcoord():
      pos()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pos.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pos.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "msg_crtcoord/crtcoord"; };
    virtual const char * getMD5() override { return "eaa03606797710dabc5cff977422c68b"; };

  };

}
#endif
