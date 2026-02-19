/*
 * Компасы
 * Pololu AltIMU-10 v5 Gyro, Accelerometer, Compass, and Altimeter (LSM6DS33, LIS3MDL, and LPS25H Carrier)
 * QMC5883L
 * GY-521 MPU6050 (9 Axis Gyroscope) + DMP (Цифровой процессор движения, Digital Motion Processor)
 * GY-521 инициализируется очень медленно (до 10 с.)
 *
 * V 1.12
 *
 * 06.03.2021
 * 14.07.2021, 26.03.2023, 11.08.2023
 * LP 21.04.2025
 */

#ifndef _YARP_COMPASS_H_
#define _YARP_COMPASS_H_

#include <Wire.h>

#include <LSM6.h>
#include <LIS3MDL.h>

#include <QMC5883L.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"

class TCompass
{
protected:
    byte ok;
    float A, K;
    int16_t _pitch, _roll;

public:
  virtual int begin(int) = 0;
  virtual int read(void) = 0;
  virtual void calibr_step(void) = 0;
  virtual uint8_t ready(void) = 0;
  void read_pitch_roll(int16_t &pitch, int16_t &roll) { pitch = _pitch; roll = _roll; }
};

class TALTIMUCompass: public TCompass
{
  private:
    LSM6 gyro_acc;
    LIS3MDL mag;
    float magnetic_field[3];
    float mag_calib[6];

    void _apply_offsets()
    {
      magnetic_field[0] = (float)(mag.m.x - mag_calib[0]) / (mag_calib[1] - mag_calib[0]) * 2 - 1;
      magnetic_field[1] = (float)(mag.m.y - mag_calib[2]) / (mag_calib[3] - mag_calib[2]) * 2 - 1;
      magnetic_field[2] = (float)(mag.m.z - mag_calib[4]) / (mag_calib[5] - mag_calib[4]) * 2 - 1;
    }
    void _magnetometer_init()
    {
      mag.init();
      mag.enableDefault();
    }
    void _get_calibration()
    {
      mag_calib[0] = min(mag_calib[0], mag.m.x);
      mag_calib[1] = max(mag_calib[1], mag.m.x);
      mag_calib[2] = min(mag_calib[2], mag.m.y);
      mag_calib[3] = max(mag_calib[3], mag.m.y);
      mag_calib[4] = min(mag_calib[4], mag.m.z);
      mag_calib[5] = max(mag_calib[5], mag.m.z);
    }
    int _read_heading()
    {
      float compass_heading;
      compass_heading = atan2(magnetic_field[1],magnetic_field[0]);
      if (compass_heading < 0)
        compass_heading += 2*PI;
      if (compass_heading >2*PI)
        compass_heading -= 2*PI;
      compass_heading *= 180.0/PI;
      return (int)compass_heading;
    }
  public:
    TALTIMUCompass(float k_lpf=1)
    // k_lpf - коэффициент фильтра низких частот
    {
      ok = 0;
      A = 0;
      K = k_lpf; //0.2
      _pitch = _roll = 0;
      for(byte i = 0; i < 6; i++)
      {
        if (i%2 != 0)
          mag_calib[i] = -32767;
        else
          mag_calib[i] = 32767;
        magnetic_field[i/2] = 0;
      }
    }
    virtual int read(void)
    {
      if(!ok) return -1;
      mag.read();
      _apply_offsets();
      int heading = _read_heading();
      A = (1.0-K)*A + K*heading;
      return (int)A;
    }
    virtual int begin(int pin_interrupt = -1)
    {
      _magnetometer_init();
      ok = 1;
      return 1;
    }
    virtual void calibr_step(void)
    {
      mag.read();
      _get_calibration();
    }
    virtual uint8_t ready(void) { return 1; }
};


class TQM5883Compass: public TCompass
{
  private:
    QMC5883L df_compass;
  public:
    TQM5883Compass(float k_lpf=1)
    // k_lpf - коэффициент фильтра низких частот
    {
      ok = 0;
      A = 0;
      K = k_lpf; //0.25
      _pitch = _roll = 0;
    }
    virtual int read(void)
    {
      if(!ok) return -1;
      if(df_compass.ready())
      {
        int heading = df_compass.readHeading();
        A = (1.0-K)*A + K*heading;
      }
      return (int)A;
    }
    virtual int begin(int pin_interrupt = -1)
    {
      df_compass.init();
      df_compass.setSamplingRate(50);
      // Пробуем
      for(byte n=0; n<10; n++)
      {
        ok = df_compass.ready();
        if(ok) break;
        delay(100);
      }
      return ok;
    }
    virtual void calibr_step(void) {}
    virtual uint8_t ready(void) { return 1; }
};

// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; }

class TMPU6050DMPCompass: public TCompass
{
  private:
    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;
    //MPU6050 mpu(0x69); // <-- use for AD0 high

    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
    // get expected DMP packet size for later comparison (default is 42 bytes)
    uint16_t packetSize;
    // return status after each device operation (0 = success, !0 = error)
    uint8_t devStatus;

  public:
    TMPU6050DMPCompass(float k_lpf=1)
    // k_lpf - коэффициент фильтра низких частот
    {
      ok = 0;
      A = 0;
      K = k_lpf; //0.25
    }
    virtual int read(void)
    {
      if(!ok) return -1;
      if(ready())
      {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        int heading = ypr[0] * 180.0 / M_PI;
        if(heading<0) heading = 360 + heading;
        _pitch = (int)(ypr[1] * 180.0 / M_PI);
        if(_pitch<0) _pitch = 360 + _pitch;
        _roll = (int)(ypr[2] * 180.0 / M_PI);
        if(_roll<0) _roll = 360 + _roll;

        A = (1.0-K)*A + K*heading;
      }
      return (int)A;
    }
    
    // Это дополнительная функция, совсем не виртуальная.
    // Сугубо рабочий объект
    int readYawPitchRoll(float fypr[])
    {
      if(!ok) return -1;
      if(ready())
      {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(fypr, &q, &gravity);
        return 1;
      }
      return 0;
    }

    virtual int begin(int pin_interrupt = -1)
    {
      ok = 0;
      // initialize device
      mpu.initialize();
      if(pin_interrupt>=0)
        pinMode(pin_interrupt, INPUT);

      // return status after each device operation (0 = success, !0 = error)
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(51);
      mpu.setYGyroOffset(8);
      mpu.setZGyroOffset(21);
      mpu.setXAccelOffset(1150);
      mpu.setYAccelOffset(-50);
      mpu.setZAccelOffset(1060);
      // make sure it worked (returns 0 if so)
      if (devStatus == 0) 
      {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        //mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        if(pin_interrupt>=0)
          attachInterrupt(digitalPinToInterrupt(pin_interrupt), dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus(); // holds actual interrupt status byte from MPU

        // get expected DMP packet size for later comparison (default is 42 bytes)
        packetSize = mpu.dmpGetFIFOPacketSize();
      }
      else
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        return 0;
      ok = 1;
      return ok;
    }

    virtual void calibr_step(void) {}
    uint8_t ready(void) { return  mpu.dmpGetCurrentFIFOPacket(fifoBuffer); }
};


#endif
