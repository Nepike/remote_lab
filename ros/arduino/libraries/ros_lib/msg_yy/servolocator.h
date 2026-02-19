#ifndef _ROS_msg_yy_servolocator_h
#define _ROS_msg_yy_servolocator_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msg_yy
{

  class servolocator : public ros::Msg
  {
    public:
      uint32_t data_length;
      typedef int16_t _data_type;
      _data_type st_data;
      _data_type * data;
      typedef int16_t _A_type;
      _A_type A;
      typedef int16_t _Idx_type;
      _Idx_type Idx;
      typedef int16_t _Val_type;
      _Val_type Val;
      typedef int16_t _A1_type;
      _A1_type A1;
      typedef int16_t _A2_type;
      _A2_type A2;
      typedef int16_t _range_min_type;
      _range_min_type range_min;
      typedef int16_t _range_max_type;
      _range_max_type range_max;

    servolocator():
      data_length(0), st_data(), data(nullptr),
      A(0),
      Idx(0),
      Val(0),
      A1(0),
      A2(0),
      range_min(0),
      range_max(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_A;
      u_A.real = this->A;
      *(outbuffer + offset + 0) = (u_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A);
      union {
        int16_t real;
        uint16_t base;
      } u_Idx;
      u_Idx.real = this->Idx;
      *(outbuffer + offset + 0) = (u_Idx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Idx.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Idx);
      union {
        int16_t real;
        uint16_t base;
      } u_Val;
      u_Val.real = this->Val;
      *(outbuffer + offset + 0) = (u_Val.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Val.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Val);
      union {
        int16_t real;
        uint16_t base;
      } u_A1;
      u_A1.real = this->A1;
      *(outbuffer + offset + 0) = (u_A1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A1);
      union {
        int16_t real;
        uint16_t base;
      } u_A2;
      u_A2.real = this->A2;
      *(outbuffer + offset + 0) = (u_A2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A2);
      union {
        int16_t real;
        uint16_t base;
      } u_range_min;
      u_range_min.real = this->range_min;
      *(outbuffer + offset + 0) = (u_range_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_min.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->range_min);
      union {
        int16_t real;
        uint16_t base;
      } u_range_max;
      u_range_max.real = this->range_max;
      *(outbuffer + offset + 0) = (u_range_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range_max.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->range_max);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (int16_t*)realloc(this->data, data_lengthT * sizeof(int16_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int16_t));
      }
      union {
        int16_t real;
        uint16_t base;
      } u_A;
      u_A.base = 0;
      u_A.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->A = u_A.real;
      offset += sizeof(this->A);
      union {
        int16_t real;
        uint16_t base;
      } u_Idx;
      u_Idx.base = 0;
      u_Idx.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Idx.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Idx = u_Idx.real;
      offset += sizeof(this->Idx);
      union {
        int16_t real;
        uint16_t base;
      } u_Val;
      u_Val.base = 0;
      u_Val.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Val.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Val = u_Val.real;
      offset += sizeof(this->Val);
      union {
        int16_t real;
        uint16_t base;
      } u_A1;
      u_A1.base = 0;
      u_A1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->A1 = u_A1.real;
      offset += sizeof(this->A1);
      union {
        int16_t real;
        uint16_t base;
      } u_A2;
      u_A2.base = 0;
      u_A2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->A2 = u_A2.real;
      offset += sizeof(this->A2);
      union {
        int16_t real;
        uint16_t base;
      } u_range_min;
      u_range_min.base = 0;
      u_range_min.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_min.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->range_min = u_range_min.real;
      offset += sizeof(this->range_min);
      union {
        int16_t real;
        uint16_t base;
      } u_range_max;
      u_range_max.base = 0;
      u_range_max.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range_max.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->range_max = u_range_max.real;
      offset += sizeof(this->range_max);
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/servolocator"; };
    virtual const char * getMD5() override { return "e19aae6d1bc8e1f26358d212ce9ab2ad"; };

  };

}
#endif
