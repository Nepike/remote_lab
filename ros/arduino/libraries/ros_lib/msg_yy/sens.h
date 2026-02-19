#ifndef _ROS_msg_yy_sens_h
#define _ROS_msg_yy_sens_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace msg_yy
{

  class sens : public ros::Msg
  {
    public:
      typedef ros::Time _tm_type;
      _tm_type tm;
      int16_t adc[10];
      int16_t data[8];
      typedef int32_t _enc_left_type;
      _enc_left_type enc_left;
      typedef int32_t _enc_right_type;
      _enc_right_type enc_right;
      typedef int16_t _compass_type;
      _compass_type compass;
      typedef int16_t _cpitch_type;
      _cpitch_type cpitch;
      typedef int16_t _croll_type;
      _croll_type croll;
      typedef float _acc_voltage_type;
      _acc_voltage_type acc_voltage;
      typedef int16_t _rf_center_type;
      _rf_center_type rf_center;
      typedef int16_t _rf_left_type;
      _rf_left_type rf_left;
      typedef int16_t _rf_right_type;
      _rf_right_type rf_right;
      typedef int16_t _rf_side_left_fwd_type;
      _rf_side_left_fwd_type rf_side_left_fwd;
      typedef int16_t _rf_side_right_fwd_type;
      _rf_side_right_fwd_type rf_side_right_fwd;
      typedef int16_t _rf_side_left_bck_type;
      _rf_side_left_bck_type rf_side_left_bck;
      typedef int16_t _rf_side_right_bck_type;
      _rf_side_right_bck_type rf_side_right_bck;
      typedef int16_t _rf_bck_center_type;
      _rf_bck_center_type rf_bck_center;
      typedef int16_t _status_type;
      _status_type status;
      int8_t switches[8];
      int32_t irdata[4];

    sens():
      tm(),
      adc(),
      data(),
      enc_left(0),
      enc_right(0),
      compass(0),
      cpitch(0),
      croll(0),
      acc_voltage(0),
      rf_center(0),
      rf_left(0),
      rf_right(0),
      rf_side_left_fwd(0),
      rf_side_right_fwd(0),
      rf_side_left_bck(0),
      rf_side_right_bck(0),
      rf_bck_center(0),
      status(0),
      switches(),
      irdata()
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
      for( uint32_t i = 0; i < 10; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_adci;
      u_adci.real = this->adc[i];
      *(outbuffer + offset + 0) = (u_adci.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adci.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc[i]);
      }
      for( uint32_t i = 0; i < 8; i++){
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
        int32_t real;
        uint32_t base;
      } u_enc_left;
      u_enc_left.real = this->enc_left;
      *(outbuffer + offset + 0) = (u_enc_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_enc_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_enc_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_enc_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->enc_left);
      union {
        int32_t real;
        uint32_t base;
      } u_enc_right;
      u_enc_right.real = this->enc_right;
      *(outbuffer + offset + 0) = (u_enc_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_enc_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_enc_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_enc_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->enc_right);
      union {
        int16_t real;
        uint16_t base;
      } u_compass;
      u_compass.real = this->compass;
      *(outbuffer + offset + 0) = (u_compass.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_compass.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->compass);
      union {
        int16_t real;
        uint16_t base;
      } u_cpitch;
      u_cpitch.real = this->cpitch;
      *(outbuffer + offset + 0) = (u_cpitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpitch.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cpitch);
      union {
        int16_t real;
        uint16_t base;
      } u_croll;
      u_croll.real = this->croll;
      *(outbuffer + offset + 0) = (u_croll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_croll.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->croll);
      union {
        float real;
        uint32_t base;
      } u_acc_voltage;
      u_acc_voltage.real = this->acc_voltage;
      *(outbuffer + offset + 0) = (u_acc_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_center;
      u_rf_center.real = this->rf_center;
      *(outbuffer + offset + 0) = (u_rf_center.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_center.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_center);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_left;
      u_rf_left.real = this->rf_left;
      *(outbuffer + offset + 0) = (u_rf_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_right;
      u_rf_right.real = this->rf_right;
      *(outbuffer + offset + 0) = (u_rf_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_left_fwd;
      u_rf_side_left_fwd.real = this->rf_side_left_fwd;
      *(outbuffer + offset + 0) = (u_rf_side_left_fwd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_side_left_fwd.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_side_left_fwd);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_right_fwd;
      u_rf_side_right_fwd.real = this->rf_side_right_fwd;
      *(outbuffer + offset + 0) = (u_rf_side_right_fwd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_side_right_fwd.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_side_right_fwd);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_left_bck;
      u_rf_side_left_bck.real = this->rf_side_left_bck;
      *(outbuffer + offset + 0) = (u_rf_side_left_bck.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_side_left_bck.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_side_left_bck);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_right_bck;
      u_rf_side_right_bck.real = this->rf_side_right_bck;
      *(outbuffer + offset + 0) = (u_rf_side_right_bck.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_side_right_bck.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_side_right_bck);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_bck_center;
      u_rf_bck_center.real = this->rf_bck_center;
      *(outbuffer + offset + 0) = (u_rf_bck_center.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rf_bck_center.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rf_bck_center);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_switchesi;
      u_switchesi.real = this->switches[i];
      *(outbuffer + offset + 0) = (u_switchesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switches[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_irdatai;
      u_irdatai.real = this->irdata[i];
      *(outbuffer + offset + 0) = (u_irdatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_irdatai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_irdatai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_irdatai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->irdata[i]);
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
      for( uint32_t i = 0; i < 10; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_adci;
      u_adci.base = 0;
      u_adci.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adci.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc[i] = u_adci.real;
      offset += sizeof(this->adc[i]);
      }
      for( uint32_t i = 0; i < 8; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_enc_left;
      u_enc_left.base = 0;
      u_enc_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_enc_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_enc_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_enc_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->enc_left = u_enc_left.real;
      offset += sizeof(this->enc_left);
      union {
        int32_t real;
        uint32_t base;
      } u_enc_right;
      u_enc_right.base = 0;
      u_enc_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_enc_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_enc_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_enc_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->enc_right = u_enc_right.real;
      offset += sizeof(this->enc_right);
      union {
        int16_t real;
        uint16_t base;
      } u_compass;
      u_compass.base = 0;
      u_compass.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_compass.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->compass = u_compass.real;
      offset += sizeof(this->compass);
      union {
        int16_t real;
        uint16_t base;
      } u_cpitch;
      u_cpitch.base = 0;
      u_cpitch.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpitch.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cpitch = u_cpitch.real;
      offset += sizeof(this->cpitch);
      union {
        int16_t real;
        uint16_t base;
      } u_croll;
      u_croll.base = 0;
      u_croll.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_croll.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->croll = u_croll.real;
      offset += sizeof(this->croll);
      union {
        float real;
        uint32_t base;
      } u_acc_voltage;
      u_acc_voltage.base = 0;
      u_acc_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_voltage = u_acc_voltage.real;
      offset += sizeof(this->acc_voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_center;
      u_rf_center.base = 0;
      u_rf_center.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_center.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_center = u_rf_center.real;
      offset += sizeof(this->rf_center);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_left;
      u_rf_left.base = 0;
      u_rf_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_left = u_rf_left.real;
      offset += sizeof(this->rf_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_right;
      u_rf_right.base = 0;
      u_rf_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_right = u_rf_right.real;
      offset += sizeof(this->rf_right);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_left_fwd;
      u_rf_side_left_fwd.base = 0;
      u_rf_side_left_fwd.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_side_left_fwd.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_side_left_fwd = u_rf_side_left_fwd.real;
      offset += sizeof(this->rf_side_left_fwd);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_right_fwd;
      u_rf_side_right_fwd.base = 0;
      u_rf_side_right_fwd.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_side_right_fwd.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_side_right_fwd = u_rf_side_right_fwd.real;
      offset += sizeof(this->rf_side_right_fwd);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_left_bck;
      u_rf_side_left_bck.base = 0;
      u_rf_side_left_bck.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_side_left_bck.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_side_left_bck = u_rf_side_left_bck.real;
      offset += sizeof(this->rf_side_left_bck);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_side_right_bck;
      u_rf_side_right_bck.base = 0;
      u_rf_side_right_bck.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_side_right_bck.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_side_right_bck = u_rf_side_right_bck.real;
      offset += sizeof(this->rf_side_right_bck);
      union {
        int16_t real;
        uint16_t base;
      } u_rf_bck_center;
      u_rf_bck_center.base = 0;
      u_rf_bck_center.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rf_bck_center.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rf_bck_center = u_rf_bck_center.real;
      offset += sizeof(this->rf_bck_center);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
      for( uint32_t i = 0; i < 8; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_switchesi;
      u_switchesi.base = 0;
      u_switchesi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switches[i] = u_switchesi.real;
      offset += sizeof(this->switches[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_irdatai;
      u_irdatai.base = 0;
      u_irdatai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_irdatai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_irdatai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_irdatai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->irdata[i] = u_irdatai.real;
      offset += sizeof(this->irdata[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/sens"; };
    virtual const char * getMD5() override { return "252fb8c7b2c55a5573258e4780359923"; };

  };

}
#endif
