#ifndef _ROS_msg_user_control_Command_h
#define _ROS_msg_user_control_Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_user_control
{

  class Command : public ros::Msg
  {
    public:
      typedef uint32_t _command_id_type;
      _command_id_type command_id;
      typedef float _priority_type;
      _priority_type priority;

    Command():
      command_id(0),
      priority(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->command_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_id);
      offset += serializeAvrFloat64(outbuffer + offset, this->priority);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->command_id =  ((uint32_t) (*(inbuffer + offset)));
      this->command_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->command_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->command_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->command_id);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->priority));
     return offset;
    }

    virtual const char * getType() override { return "msg_user_control/Command"; };
    virtual const char * getMD5() override { return "a11be0b5c2c31c516b0e7e0a6eed0c73"; };

  };

}
#endif
