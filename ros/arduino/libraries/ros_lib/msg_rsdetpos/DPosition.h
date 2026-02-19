#ifndef _ROS_msg_rsdetpos_DPosition_h
#define _ROS_msg_rsdetpos_DPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Odometry.h"

namespace msg_rsdetpos
{

  class DPosition : public ros::Msg
  {
    public:
      typedef nav_msgs::Odometry _pos_type;
      _pos_type pos;
      typedef int32_t _robot_id_type;
      _robot_id_type robot_id;
      typedef int32_t _team_id_type;
      _team_id_type team_id;

    DPosition():
      pos(),
      robot_id(0),
      team_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pos.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_robot_id;
      u_robot_id.real = this->robot_id;
      *(outbuffer + offset + 0) = (u_robot_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_id);
      union {
        int32_t real;
        uint32_t base;
      } u_team_id;
      u_team_id.real = this->team_id;
      *(outbuffer + offset + 0) = (u_team_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_team_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_team_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_team_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->team_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pos.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_robot_id;
      u_robot_id.base = 0;
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_id = u_robot_id.real;
      offset += sizeof(this->robot_id);
      union {
        int32_t real;
        uint32_t base;
      } u_team_id;
      u_team_id.base = 0;
      u_team_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_team_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_team_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_team_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->team_id = u_team_id.real;
      offset += sizeof(this->team_id);
     return offset;
    }

    virtual const char * getType() override { return "msg_rsdetpos/DPosition"; };
    virtual const char * getMD5() override { return "4475d99a4258351b628d2c18545b564b"; };

  };

}
#endif
