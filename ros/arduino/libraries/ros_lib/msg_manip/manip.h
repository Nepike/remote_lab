#ifndef _ROS_msg_manip_manip_h
#define _ROS_msg_manip_manip_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace msg_manip
{

  class manip : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _command_type;
      _command_type command;
      typedef int32_t _manip_id_type;
      _manip_id_type manip_id;
      typedef int32_t _x_type;
      _x_type x;
      typedef int32_t _y_type;
      _y_type y;
      typedef int32_t _z_type;
      _z_type z;
      typedef int8_t _catch_stat_type;
      _catch_stat_type catch_stat;
      typedef int8_t _chain_num_type;
      _chain_num_type chain_num;
      typedef int32_t _ang_val_type;
      _ang_val_type ang_val;
      typedef int32_t _itter_num_type;
      _itter_num_type itter_num;
      typedef int32_t _dist_type;
      _dist_type dist;
      typedef uint32_t _sleep_time_type;
      _sleep_time_type sleep_time;

    manip():
      header(),
      command(""),
      manip_id(0),
      x(0),
      y(0),
      z(0),
      catch_stat(0),
      chain_num(0),
      ang_val(0),
      itter_num(0),
      dist(0),
      sleep_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      union {
        int32_t real;
        uint32_t base;
      } u_manip_id;
      u_manip_id.real = this->manip_id;
      *(outbuffer + offset + 0) = (u_manip_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_manip_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_manip_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_manip_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->manip_id);
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
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        int8_t real;
        uint8_t base;
      } u_catch_stat;
      u_catch_stat.real = this->catch_stat;
      *(outbuffer + offset + 0) = (u_catch_stat.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->catch_stat);
      union {
        int8_t real;
        uint8_t base;
      } u_chain_num;
      u_chain_num.real = this->chain_num;
      *(outbuffer + offset + 0) = (u_chain_num.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->chain_num);
      union {
        int32_t real;
        uint32_t base;
      } u_ang_val;
      u_ang_val.real = this->ang_val;
      *(outbuffer + offset + 0) = (u_ang_val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ang_val.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ang_val.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ang_val.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ang_val);
      union {
        int32_t real;
        uint32_t base;
      } u_itter_num;
      u_itter_num.real = this->itter_num;
      *(outbuffer + offset + 0) = (u_itter_num.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_itter_num.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_itter_num.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_itter_num.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->itter_num);
      union {
        int32_t real;
        uint32_t base;
      } u_dist;
      u_dist.real = this->dist;
      *(outbuffer + offset + 0) = (u_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dist.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dist.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dist.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dist);
      *(outbuffer + offset + 0) = (this->sleep_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sleep_time >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sleep_time >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sleep_time >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sleep_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      union {
        int32_t real;
        uint32_t base;
      } u_manip_id;
      u_manip_id.base = 0;
      u_manip_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_manip_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_manip_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_manip_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->manip_id = u_manip_id.real;
      offset += sizeof(this->manip_id);
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
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        int8_t real;
        uint8_t base;
      } u_catch_stat;
      u_catch_stat.base = 0;
      u_catch_stat.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->catch_stat = u_catch_stat.real;
      offset += sizeof(this->catch_stat);
      union {
        int8_t real;
        uint8_t base;
      } u_chain_num;
      u_chain_num.base = 0;
      u_chain_num.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->chain_num = u_chain_num.real;
      offset += sizeof(this->chain_num);
      union {
        int32_t real;
        uint32_t base;
      } u_ang_val;
      u_ang_val.base = 0;
      u_ang_val.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ang_val.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ang_val.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ang_val.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ang_val = u_ang_val.real;
      offset += sizeof(this->ang_val);
      union {
        int32_t real;
        uint32_t base;
      } u_itter_num;
      u_itter_num.base = 0;
      u_itter_num.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_itter_num.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_itter_num.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_itter_num.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->itter_num = u_itter_num.real;
      offset += sizeof(this->itter_num);
      union {
        int32_t real;
        uint32_t base;
      } u_dist;
      u_dist.base = 0;
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dist = u_dist.real;
      offset += sizeof(this->dist);
      this->sleep_time =  ((uint32_t) (*(inbuffer + offset)));
      this->sleep_time |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sleep_time |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sleep_time |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sleep_time);
     return offset;
    }

    virtual const char * getType() override { return "msg_manip/manip"; };
    virtual const char * getMD5() override { return "a7fd36dac1131a0cdea251ffb061bca1"; };

  };

}
#endif
