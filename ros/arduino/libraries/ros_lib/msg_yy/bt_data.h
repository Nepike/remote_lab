#ifndef _ROS_msg_yy_bt_data_h
#define _ROS_msg_yy_bt_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace msg_yy
{

  class bt_data : public ros::Msg
  {
    public:
      typedef ros::Time _tm_type;
      _tm_type tm;
      typedef float _AccX_type;
      _AccX_type AccX;
      typedef float _AccY_type;
      _AccY_type AccY;
      typedef float _AccZ_type;
      _AccZ_type AccZ;
      typedef float _AngVelX_type;
      _AngVelX_type AngVelX;
      typedef float _AngVelY_type;
      _AngVelY_type AngVelY;
      typedef float _AngVelZ_type;
      _AngVelZ_type AngVelZ;
      typedef float _Azimuth_type;
      _Azimuth_type Azimuth;
      typedef float _Pitch_type;
      _Pitch_type Pitch;
      typedef float _Roll_type;
      _Roll_type Roll;
      typedef float _Distance_type;
      _Distance_type Distance;
      typedef float _Latitude_type;
      _Latitude_type Latitude;
      typedef float _Longitude_type;
      _Longitude_type Longitude;
      typedef float _Altitude_type;
      _Altitude_type Altitude;
      typedef float _Speed_type;
      _Speed_type Speed;

    bt_data():
      tm(),
      AccX(0),
      AccY(0),
      AccZ(0),
      AngVelX(0),
      AngVelY(0),
      AngVelZ(0),
      Azimuth(0),
      Pitch(0),
      Roll(0),
      Distance(0),
      Latitude(0),
      Longitude(0),
      Altitude(0),
      Speed(0)
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
      union {
        float real;
        uint32_t base;
      } u_AccX;
      u_AccX.real = this->AccX;
      *(outbuffer + offset + 0) = (u_AccX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AccX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AccX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AccX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AccX);
      union {
        float real;
        uint32_t base;
      } u_AccY;
      u_AccY.real = this->AccY;
      *(outbuffer + offset + 0) = (u_AccY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AccY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AccY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AccY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AccY);
      union {
        float real;
        uint32_t base;
      } u_AccZ;
      u_AccZ.real = this->AccZ;
      *(outbuffer + offset + 0) = (u_AccZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AccZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AccZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AccZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AccZ);
      union {
        float real;
        uint32_t base;
      } u_AngVelX;
      u_AngVelX.real = this->AngVelX;
      *(outbuffer + offset + 0) = (u_AngVelX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelX);
      union {
        float real;
        uint32_t base;
      } u_AngVelY;
      u_AngVelY.real = this->AngVelY;
      *(outbuffer + offset + 0) = (u_AngVelY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelY);
      union {
        float real;
        uint32_t base;
      } u_AngVelZ;
      u_AngVelZ.real = this->AngVelZ;
      *(outbuffer + offset + 0) = (u_AngVelZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_AngVelZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_AngVelZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_AngVelZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->AngVelZ);
      union {
        float real;
        uint32_t base;
      } u_Azimuth;
      u_Azimuth.real = this->Azimuth;
      *(outbuffer + offset + 0) = (u_Azimuth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Azimuth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Azimuth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Azimuth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Azimuth);
      union {
        float real;
        uint32_t base;
      } u_Pitch;
      u_Pitch.real = this->Pitch;
      *(outbuffer + offset + 0) = (u_Pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Pitch);
      union {
        float real;
        uint32_t base;
      } u_Roll;
      u_Roll.real = this->Roll;
      *(outbuffer + offset + 0) = (u_Roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Roll);
      union {
        float real;
        uint32_t base;
      } u_Distance;
      u_Distance.real = this->Distance;
      *(outbuffer + offset + 0) = (u_Distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Distance);
      union {
        float real;
        uint32_t base;
      } u_Latitude;
      u_Latitude.real = this->Latitude;
      *(outbuffer + offset + 0) = (u_Latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Latitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Latitude);
      union {
        float real;
        uint32_t base;
      } u_Longitude;
      u_Longitude.real = this->Longitude;
      *(outbuffer + offset + 0) = (u_Longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Longitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Longitude);
      union {
        float real;
        uint32_t base;
      } u_Altitude;
      u_Altitude.real = this->Altitude;
      *(outbuffer + offset + 0) = (u_Altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Altitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Altitude);
      union {
        float real;
        uint32_t base;
      } u_Speed;
      u_Speed.real = this->Speed;
      *(outbuffer + offset + 0) = (u_Speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Speed);
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
      union {
        float real;
        uint32_t base;
      } u_AccX;
      u_AccX.base = 0;
      u_AccX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AccX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AccX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AccX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AccX = u_AccX.real;
      offset += sizeof(this->AccX);
      union {
        float real;
        uint32_t base;
      } u_AccY;
      u_AccY.base = 0;
      u_AccY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AccY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AccY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AccY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AccY = u_AccY.real;
      offset += sizeof(this->AccY);
      union {
        float real;
        uint32_t base;
      } u_AccZ;
      u_AccZ.base = 0;
      u_AccZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AccZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AccZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AccZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AccZ = u_AccZ.real;
      offset += sizeof(this->AccZ);
      union {
        float real;
        uint32_t base;
      } u_AngVelX;
      u_AngVelX.base = 0;
      u_AngVelX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelX = u_AngVelX.real;
      offset += sizeof(this->AngVelX);
      union {
        float real;
        uint32_t base;
      } u_AngVelY;
      u_AngVelY.base = 0;
      u_AngVelY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelY = u_AngVelY.real;
      offset += sizeof(this->AngVelY);
      union {
        float real;
        uint32_t base;
      } u_AngVelZ;
      u_AngVelZ.base = 0;
      u_AngVelZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_AngVelZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_AngVelZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_AngVelZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->AngVelZ = u_AngVelZ.real;
      offset += sizeof(this->AngVelZ);
      union {
        float real;
        uint32_t base;
      } u_Azimuth;
      u_Azimuth.base = 0;
      u_Azimuth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Azimuth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Azimuth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Azimuth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Azimuth = u_Azimuth.real;
      offset += sizeof(this->Azimuth);
      union {
        float real;
        uint32_t base;
      } u_Pitch;
      u_Pitch.base = 0;
      u_Pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Pitch = u_Pitch.real;
      offset += sizeof(this->Pitch);
      union {
        float real;
        uint32_t base;
      } u_Roll;
      u_Roll.base = 0;
      u_Roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Roll = u_Roll.real;
      offset += sizeof(this->Roll);
      union {
        float real;
        uint32_t base;
      } u_Distance;
      u_Distance.base = 0;
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Distance = u_Distance.real;
      offset += sizeof(this->Distance);
      union {
        float real;
        uint32_t base;
      } u_Latitude;
      u_Latitude.base = 0;
      u_Latitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Latitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Latitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Latitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Latitude = u_Latitude.real;
      offset += sizeof(this->Latitude);
      union {
        float real;
        uint32_t base;
      } u_Longitude;
      u_Longitude.base = 0;
      u_Longitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Longitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Longitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Longitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Longitude = u_Longitude.real;
      offset += sizeof(this->Longitude);
      union {
        float real;
        uint32_t base;
      } u_Altitude;
      u_Altitude.base = 0;
      u_Altitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Altitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Altitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Altitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Altitude = u_Altitude.real;
      offset += sizeof(this->Altitude);
      union {
        float real;
        uint32_t base;
      } u_Speed;
      u_Speed.base = 0;
      u_Speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Speed = u_Speed.real;
      offset += sizeof(this->Speed);
     return offset;
    }

    virtual const char * getType() override { return "msg_yy/bt_data"; };
    virtual const char * getMD5() override { return "01ab12034a894209c809613865cc838f"; };

  };

}
#endif
