#ifndef _ROS_msg_kvorum2_action_h
#define _ROS_msg_kvorum2_action_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_kvorum2
{

  class action : public ros::Msg
  {
    public:
      typedef int8_t _team_id_type;
      _team_id_type team_id;
      typedef int16_t _agent_id_type;
      _agent_id_type agent_id;
      typedef int16_t _action_type;
      _action_type action;
      uint32_t slist_length;
      typedef char* _slist_type;
      _slist_type st_slist;
      _slist_type * slist;
      typedef float _arg1_type;
      _arg1_type arg1;
      typedef float _arg2_type;
      _arg2_type arg2;
      typedef float _arg3_type;
      _arg3_type arg3;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;

    action():
      team_id(0),
      agent_id(0),
      action(0),
      slist_length(0), st_slist(), slist(nullptr),
      arg1(0),
      arg2(0),
      arg3(0),
      data_length(0), st_data(), data(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_team_id;
      u_team_id.real = this->team_id;
      *(outbuffer + offset + 0) = (u_team_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->team_id);
      union {
        int16_t real;
        uint16_t base;
      } u_agent_id;
      u_agent_id.real = this->agent_id;
      *(outbuffer + offset + 0) = (u_agent_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_agent_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->agent_id);
      union {
        int16_t real;
        uint16_t base;
      } u_action;
      u_action.real = this->action;
      *(outbuffer + offset + 0) = (u_action.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_action.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->action);
      *(outbuffer + offset + 0) = (this->slist_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->slist_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->slist_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->slist_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slist_length);
      for( uint32_t i = 0; i < slist_length; i++){
      uint32_t length_slisti = strlen(this->slist[i]);
      varToArr(outbuffer + offset, length_slisti);
      offset += 4;
      memcpy(outbuffer + offset, this->slist[i], length_slisti);
      offset += length_slisti;
      }
      union {
        float real;
        uint32_t base;
      } u_arg1;
      u_arg1.real = this->arg1;
      *(outbuffer + offset + 0) = (u_arg1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg1);
      union {
        float real;
        uint32_t base;
      } u_arg2;
      u_arg2.real = this->arg2;
      *(outbuffer + offset + 0) = (u_arg2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg2);
      union {
        float real;
        uint32_t base;
      } u_arg3;
      u_arg3.real = this->arg3;
      *(outbuffer + offset + 0) = (u_arg3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arg3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arg3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arg3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg3);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_team_id;
      u_team_id.base = 0;
      u_team_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->team_id = u_team_id.real;
      offset += sizeof(this->team_id);
      union {
        int16_t real;
        uint16_t base;
      } u_agent_id;
      u_agent_id.base = 0;
      u_agent_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_agent_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->agent_id = u_agent_id.real;
      offset += sizeof(this->agent_id);
      union {
        int16_t real;
        uint16_t base;
      } u_action;
      u_action.base = 0;
      u_action.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_action.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->action = u_action.real;
      offset += sizeof(this->action);
      uint32_t slist_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      slist_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      slist_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      slist_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->slist_length);
      if(slist_lengthT > slist_length)
        this->slist = (char**)realloc(this->slist, slist_lengthT * sizeof(char*));
      slist_length = slist_lengthT;
      for( uint32_t i = 0; i < slist_length; i++){
      uint32_t length_st_slist;
      arrToVar(length_st_slist, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_slist; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_slist-1]=0;
      this->st_slist = (char *)(inbuffer + offset-1);
      offset += length_st_slist;
        memcpy( &(this->slist[i]), &(this->st_slist), sizeof(char*));
      }
      union {
        float real;
        uint32_t base;
      } u_arg1;
      u_arg1.base = 0;
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg1 = u_arg1.real;
      offset += sizeof(this->arg1);
      union {
        float real;
        uint32_t base;
      } u_arg2;
      u_arg2.base = 0;
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg2 = u_arg2.real;
      offset += sizeof(this->arg2);
      union {
        float real;
        uint32_t base;
      } u_arg3;
      u_arg3.base = 0;
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arg3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arg3 = u_arg3.real;
      offset += sizeof(this->arg3);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_kvorum2/action"; };
    virtual const char * getMD5() override { return "84a3ef5c88aaf825ce459854b370cf6c"; };

  };

}
#endif
