#ifndef _ROS_msg_mio_miodata_h
#define _ROS_msg_mio_miodata_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace msg_mio
{

  class miodata : public ros::Msg
  {
    public:
      typedef ros::Time _tm_type;
      _tm_type tm;
      uint32_t rawdata_length;
      typedef int16_t _rawdata_type;
      _rawdata_type st_rawdata;
      _rawdata_type * rawdata;
      uint32_t fdata_length;
      typedef int16_t _fdata_type;
      _fdata_type st_fdata;
      _fdata_type * fdata;
      uint32_t sign_length;
      typedef int8_t _sign_type;
      _sign_type st_sign;
      _sign_type * sign;

    miodata():
      tm(),
      rawdata_length(0), st_rawdata(), rawdata(nullptr),
      fdata_length(0), st_fdata(), fdata(nullptr),
      sign_length(0), st_sign(), sign(nullptr)
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
      *(outbuffer + offset + 0) = (this->rawdata_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rawdata_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rawdata_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rawdata_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rawdata_length);
      for( uint32_t i = 0; i < rawdata_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_rawdatai;
      u_rawdatai.real = this->rawdata[i];
      *(outbuffer + offset + 0) = (u_rawdatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rawdatai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rawdata[i]);
      }
      *(outbuffer + offset + 0) = (this->fdata_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fdata_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fdata_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fdata_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fdata_length);
      for( uint32_t i = 0; i < fdata_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_fdatai;
      u_fdatai.real = this->fdata[i];
      *(outbuffer + offset + 0) = (u_fdatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fdatai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->fdata[i]);
      }
      *(outbuffer + offset + 0) = (this->sign_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sign_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sign_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sign_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sign_length);
      for( uint32_t i = 0; i < sign_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_signi;
      u_signi.real = this->sign[i];
      *(outbuffer + offset + 0) = (u_signi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sign[i]);
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
      uint32_t rawdata_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rawdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rawdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rawdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rawdata_length);
      if(rawdata_lengthT > rawdata_length)
        this->rawdata = (int16_t*)realloc(this->rawdata, rawdata_lengthT * sizeof(int16_t));
      rawdata_length = rawdata_lengthT;
      for( uint32_t i = 0; i < rawdata_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_rawdata;
      u_st_rawdata.base = 0;
      u_st_rawdata.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rawdata.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_rawdata = u_st_rawdata.real;
      offset += sizeof(this->st_rawdata);
        memcpy( &(this->rawdata[i]), &(this->st_rawdata), sizeof(int16_t));
      }
      uint32_t fdata_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fdata_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fdata_length);
      if(fdata_lengthT > fdata_length)
        this->fdata = (int16_t*)realloc(this->fdata, fdata_lengthT * sizeof(int16_t));
      fdata_length = fdata_lengthT;
      for( uint32_t i = 0; i < fdata_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_fdata;
      u_st_fdata.base = 0;
      u_st_fdata.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fdata.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_fdata = u_st_fdata.real;
      offset += sizeof(this->st_fdata);
        memcpy( &(this->fdata[i]), &(this->st_fdata), sizeof(int16_t));
      }
      uint32_t sign_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sign_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sign_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sign_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sign_length);
      if(sign_lengthT > sign_length)
        this->sign = (int8_t*)realloc(this->sign, sign_lengthT * sizeof(int8_t));
      sign_length = sign_lengthT;
      for( uint32_t i = 0; i < sign_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_sign;
      u_st_sign.base = 0;
      u_st_sign.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_sign = u_st_sign.real;
      offset += sizeof(this->st_sign);
        memcpy( &(this->sign[i]), &(this->st_sign), sizeof(int8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_mio/miodata"; };
    virtual const char * getMD5() override { return "a6df59724e3f93ba3c71cfdfa85a20bc"; };

  };

}
#endif
