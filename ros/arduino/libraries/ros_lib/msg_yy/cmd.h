#ifndef _ROS_msg_yy_cmd_h
#define _ROS_msg_yy_cmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_yy
{

  class cmd : public ros::Msg
  {
    public:
      typedef int16_t _id_type;
      _id_type id;
      typedef int16_t _command_type;
      _command_type command;
      uint32_t arg_length;
      typedef float _arg_type;
      _arg_type st_arg;
      _arg_type * arg;
      uint32_t angle_length;
      typedef int16_t _angle_type;
      _angle_type st_angle;
      _angle_type * angle;
      int32_t da[4];

    cmd():
      id(0),
      command(0),
      arg_length(0), st_arg(), arg(nullptr),
      angle_length(0), st_angle(), angle(nullptr),
      da()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_command;
      u_command.real = this->command;
      *(outbuffer + offset + 0) = (u_command.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_command.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->command);
      *(outbuffer + offset + 0) = (this->arg_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arg_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arg_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arg_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg_length);
      for( uint32_t i = 0; i < arg_length; i++){
      union {
        float real;
        uint32_t base;
      } u_argi;
      u_argi.real = this->arg[i];
      *(outbuffer + offset + 0) = (u_argi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_argi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_argi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_argi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg[i]);
      }
      *(outbuffer + offset + 0) = (this->angle_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angle_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angle_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angle_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_length);
      for( uint32_t i = 0; i < angle_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_dai;
      u_dai.real = this->da[i];
      *(outbuffer + offset + 0) = (u_dai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->da[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_command;
      u_command.base = 0;
      u_command.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_command.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->command = u_command.real;
      offset += sizeof(this->command);
      uint32_t arg_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      arg_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      arg_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      arg_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->arg_length);
      if(arg_lengthT > arg_length)
        this->arg = (float*)realloc(this->arg, arg_lengthT * sizeof(float));
      arg_length = arg_lengthT;
      for( uint32_t i = 0; i < arg_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_arg;
      u_st_arg.base = 0;
      u_st_arg.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_arg.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_arg.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_arg.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_arg = u_st_arg.real;
      offset += sizeof(this->st_arg);
        memcpy( &(this->arg[i]), &(this->st_arg), sizeof(float));
      }
      uint32_t angle_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angle_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angle_length);
      if(angle_lengthT > angle_length)
        this->angle = (int16_t*)realloc(this->angle, angle_lengthT * sizeof(int16_t));
      angle_length = angle_lengthT;
      for( uint32_t i = 0; i < angle_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_angle;
      u_st_angle.base = 0;
      u_st_angle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_angle = u_st_angle.real;
      offset += sizeof(this->st_angle);
        memcpy( &(this->angle[i]), &(this->st_angle), sizeof(int16_t));
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_dai;
      u_dai.base = 0;
      u_dai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->da[i] = u_dai.real;
      offset += sizeof(this->da[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/cmd"; };
    virtual const char * getMD5() override { return "3d91a31402987055a496781bb805db1c"; };

  };

}
#endif
