#ifndef _ROS_SERVICE_SpeechCommand_h
#define _ROS_SERVICE_SpeechCommand_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "msg_speech/Speech.h"
#include "msg_user_control/Command.h"

namespace msg_speech
{

static const char SPEECHCOMMAND[] = "msg_speech/SpeechCommand";

  class SpeechCommandRequest : public ros::Msg
  {
    public:
      typedef msg_speech::Speech _alternatives_type;
      _alternatives_type alternatives;

    SpeechCommandRequest():
      alternatives()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->alternatives.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->alternatives.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SPEECHCOMMAND; };
    virtual const char * getMD5() override { return "575fa5a356d67d91b8725f67cb701074"; };

  };

  class SpeechCommandResponse : public ros::Msg
  {
    public:
      typedef msg_user_control::Command _cmd_type;
      _cmd_type cmd;

    SpeechCommandResponse():
      cmd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->cmd.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->cmd.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SPEECHCOMMAND; };
    virtual const char * getMD5() override { return "0b7d7abf140180435f9e2ec4b3c42885"; };

  };

  class SpeechCommand {
    public:
    typedef SpeechCommandRequest Request;
    typedef SpeechCommandResponse Response;
  };

}
#endif
