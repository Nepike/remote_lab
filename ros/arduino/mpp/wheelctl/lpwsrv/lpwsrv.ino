 /**
 * \file lpwheelctl.ino
 * Progect:
 *  ЛППП Управление рулевым колесом. Сервер
 *  DualVNH5019MotorShield
 *  Chip ATmega328
 *  \author Robofob
 *  \version 1.05
 *  \date 05.05.2015
 *  \date Last Change: 22.06.2015
 */

#include <Wire.h>
#include <i2cwctl.h>

#define Title "LPWheelCtl 1.05"

// Управление двигателем
#define pinPWM 6 // ШИМ вход. Левое колесо
#define pinCTL 7 // Цифровой вход

#define pinCLeft  12 // Датчик (концевик) левого крайнего положения
#define pinCRight 11 // Датчик (концевик) правого крайнего положения
#define PResistor A0 // Датчик угла поворота

//----------------------------------------------------------

#define MAX_SPEED 100  // 150 Максимальная скорость движения

int CurrentSpeed = MAX_SPEED;

int RLeft = 1023, RRight = 0, RCenter;

boolean CalibrationDone = false; /// Флаг готовности. Выставляется после выполнения процедуры калибровки 

#define MinAngle -90
#define MaxAngle  90

// Буфер для вывода сообщений
char str[60];

//----------------------------------------------------------
// Двигательные функции
//----------------------------------------------------------

void MotorGo(int speed)
{
  if(speed==0)
  {
    digitalWrite(pinCTL,HIGH);
    digitalWrite(pinPWM,HIGH);
  }
  else if(speed>0)
  {
    digitalWrite(pinCTL,LOW);
    analogWrite(pinPWM,speed);
  }
  else
  {
    digitalWrite(pinCTL,HIGH);
    analogWrite(pinPWM,speed+255);
  }
}

inline void MotorStop(void) { MotorGo(0); }
inline void goLeft(int speed) { MotorGo(speed); }
inline void goRight(int speed) { MotorGo(-speed); }

//----------------------------------------------------------------
// События: прием запросов от мастера
//----------------------------------------------------------------
byte I2C_COMMAND_READY = 0;
byte I2CCMD, I2CDATA1, I2CDATA2;
int I2CINTDATA;

void receiveI2CEvent(int howMany)
// Принимается сначала младший байт, а зетем - старший
{
  I2CCMD = Wire.read();
  I2CDATA1 = Wire.read();
  I2CDATA2 = Wire.read();
  I2CINTDATA = ((int)I2CDATA2<<8) | I2CDATA1;
  I2C_COMMAND_READY = 1;
}

// BS_IS_LEFT срабатывает, когда колесо довернуто до упора налево, BS_IS_RIGHT - когда направо
#define BS_IS_LEFT(n)  (n & 0x01)
#define BS_IS_RIGHT(n) (n & 0x10)

byte BumperStatus(void)
{
  byte clf, crt;
  clf = (digitalRead(pinCLeft)==0);
  crt = (digitalRead(pinCRight)==0);
  return clf + crt*2;
}

byte WaitForBumper(void)
{
  byte bs;
  do
  {
    bs = BumperStatus();
    delay(10);
  } while(bs==0);
  return bs;
}

void CalibrateWheel(void)
{
  int r;

  Serial.println("Calibrate proc");
  // До упора налево
  Serial.println("GoLeft");
  goLeft(CurrentSpeed);
  WaitForBumper();
  MotorStop(); 
  RLeft = analogRead(PResistor);
  
  // До упора направо
  Serial.println("GoRight");
  goRight(CurrentSpeed);
  while(BumperStatus()!=0) delay(10);
  
  WaitForBumper();
  MotorStop();
  
  RRight = analogRead(PResistor);
  RCenter = (RLeft+RRight)/2;

  snprintf(str, sizeof(str), "%d %d %d", RLeft, RRight, RCenter);
  Serial.println(str);  
  
  // В центр
  Serial.println("GoCenter");
  goLeft(CurrentSpeed);  
  do
  {
    r = analogRead(PResistor);
  } while(r<RCenter);
  MotorStop();
  Serial.println("Done");
  CalibrationDone = true;
}

/*
  Вращаем налево - напряжение на резисторе возрастает
  Вращаем направо - напряжение на резисторе уменьшается
  RLeft  - MinAngle
  RRight - MaxAngle
*/
int Angle2Pos(int angle)
{
  if(angle<MinAngle) angle=MinAngle;
  if(angle>MaxAngle) angle=MaxAngle;
  int pos = map(angle, MinAngle, MaxAngle, RLeft, RRight);
  return pos;
}

void WheelSetAngle(int angle)
{
  byte n;
  if(!CalibrationDone) return;
  
  if(angle<MinAngle) angle=MinAngle;
  if(angle>MaxAngle) angle=MaxAngle;

  int currpos = analogRead(PResistor);
  int goalpos = Angle2Pos(angle);
  enum {DIR_LEFT, DIR_RIGHT};
  byte dir, eoj;
  if(currpos<goalpos) // Надо увеличивать currpos - едем налево
    dir = DIR_LEFT;
  else
    dir = DIR_RIGHT;

  snprintf(str, sizeof(str), "Rotate %d [%d] (%d -> %d)", angle, dir, currpos, goalpos);
  Serial.println(str);

  if(dir == DIR_LEFT) // Надо увеличивать currpos - едем налево
    goLeft(CurrentSpeed);
  else
    goRight(CurrentSpeed);

  // При повороте мы не будем сравнивать с дельтой - это не очень надежно  
  eoj = 0;
  while(!eoj)
  {
    n = BumperStatus();
    currpos = analogRead(PResistor);

//    snprintf(str, sizeof(str), "n = %d, dir = %d, currpos = %d, goalpos = %d", n, dir, currpos, goalpos);
//    Serial.println(str);

    // BS_IS_LEFT срабатывает, когда колесо довернуто до упора налево, BS_IS_RIGHT - когда направо
    if(dir==DIR_LEFT) // Едем налево, увеличиваем currpos. Условие завершения: currpos>goalpos
      eoj = (currpos>=goalpos || BS_IS_LEFT(n));
    else // Едем направо, уменьшаем currpos
      eoj = (currpos<=goalpos || BS_IS_RIGHT(n));
  }
  MotorStop();
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void setup()
{  
  Wire.begin(I2CLPW::ADDR);   // join i2c bus with address I2C_LPW_CTL_ADDR
  Wire.onReceive(receiveI2CEvent);
  
  Serial.begin(9600);
  Serial.println(Title);

  pinMode(pinCTL,OUTPUT);
  pinMode(pinPWM,OUTPUT);

  pinMode(pinCLeft,INPUT);
  digitalWrite(pinCLeft,HIGH);

  pinMode(pinCRight,INPUT);
  digitalWrite(pinCRight,HIGH);

  MotorStop(); 
  // Тест
  /**
  CalibrateWheel();
  delay(1000);
  
  WheelSetAngle(-45);
  delay(3000);
  
  WheelSetAngle(45);
  delay(3000);

  WheelSetAngle(0);
  **/
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void loop()
{
  int angle;
  if(I2C_COMMAND_READY)
  {
    I2C_COMMAND_READY = 0;
    snprintf(str, sizeof(str), "> %4x %4d (%2x %2x)", I2CCMD, I2CINTDATA, I2CDATA1, I2CDATA2);
    Serial.println(str);
    
    switch(I2CCMD)
    {
      case I2CLPW::DCMD_SET_ANG:
        angle = (signed char)I2CDATA1;
        WheelSetAngle(angle);
        break;
      case I2CLPW::DCMD_CALIBRATE:
        CalibrateWheel();
        break;
      case I2CLPW::DCMD_SET_SPEED:
        if(I2CDATA1<MAX_SPEED)
          CurrentSpeed = I2CDATA1;
        break;
    }
  }
}

