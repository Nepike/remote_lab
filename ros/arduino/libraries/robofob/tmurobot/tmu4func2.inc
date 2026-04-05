/**
 * \file tmu4func2.inc
 * Progect: TMUx
 * Базовая прошивка
 *
 *  Робот с колесными модулями LLW
 *  L298 Motor Driver
 *
 *  Протокол RCX2
 *  Baudrate 9600/115200
 *  Все значения датчиков считываются в 8-разрядном формате
 *  Chip ATmega328

 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016 / 24.08.2017
 *  \date LP 28.04.2020
 */

#ifndef _TMU4FUNC2_INC_
#define _TMU4FUNC2_INC_

void Beep(byte n)
{
  for(byte i=0;i<n;i++)
  {
    digitalWrite(PIN_BEEP, HIGH);
    delay(200);
    digitalWrite(PIN_BEEP, LOW);
    delay(200);
  }
}

int ang2pos(int ang)
{
  #define MIN_PULSE  750
  #define MAX_PULSE  2250
  int pos;
  pos = map(ang, 0, 180, MIN_PULSE, MAX_PULSE);
  return pos;
}

void SetServoAng(int ang)
{
  int pos;
  pos = ang2pos(ang);
  myservo.write(pos);
}

#define AngleStep 4
#define MIN_ANG   0
#define MAX_ANG 180

#define LDNUM ((MAX_ANG-MIN_ANG+1)/AngleStep+1)
byte LocatorData[LDNUM];

void TestServo(void)
{
  SetServoAng((MIN_ANG+MAX_ANG)/2);
  delay(1000);
  SetServoAng(MIN_ANG);
  delay(1000);
  SetServoAng((MIN_ANG+MAX_ANG)/2);
  delay(1000);
  SetServoAng(MAX_ANG);
  delay(1000);
  SetServoAng((MIN_ANG+MAX_ANG)/2);
}

void ServoProc(void)
{
  static int currang = 0;
  static byte dir = 0;
  static int PRED_T_WCNT = 0;
  int nd;
  if(PRED_T_WCNT == stdrobot::T_WCNT) return;
  PRED_T_WCNT = stdrobot::T_WCNT;

  SetServoAng(currang);

  REG[REG_CURR_ANG] = (short int)currang;

  // Считываем показание дальномера
  nd = currang/AngleStep;
  if(nd>=LDNUM) nd=LDNUM-1;

  LocatorData[nd] = (analogRead(PIN_SHARP_LOCATOR)>>2);

  if(dir==0) // Увеличение
  {
    currang+=AngleStep;
    if(currang>MAX_ANG)
    {
      currang = MAX_ANG-AngleStep;
      dir = 1;
    }
  }
  else // Уменьшение
  {
    currang-=AngleStep;
    if(currang<MIN_ANG)
    {
      currang = MIN_ANG+AngleStep;
      dir = 0;
    }
  }
}

void WriteUsrData(byte addr)
// Выдача массива показаний локатора
{
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(addr);
  pckg.rcWriteBytePtr(pckg.MY_ADDR);
  pckg.rcWriteBytePtr(CMD_ANS_GET_USR_DATA);
  pckg.rcWriteBytePtr(LDNUM);
  for(byte i=0;i<LDNUM;i++)
    pckg.rcWriteBytePtr(LocatorData[i]);
  pckg.rcWriteBytePtr(CS_VALUE);
}

// ----------------------------------------------------------------
// Работа с измерением напряжения питания
// ----------------------------------------------------------------

void CheckPower(float MinVPower)
{
  const float delta = 0.13; // delta % ошибка датчика (X-Y) / X
  static int cnt = 0;
  cnt++;
  // Эта задержка измерений нужна для того, чтобы значение успело установиться
  if(cnt<250) return;
  cnt = 0;
  // Эти странные константы определяются параметрами датчика напряжения (7.5K и 30K)
  float u = Sens[SNUM_VPOWER].GetVal()*13.3/(548.0/4);
  u = u * 1 / (1 - delta); // error correction

  float cu = analogRead(PIN_VPOWER)*13.3/548.0;
  cu = cu * 1 / (1 - delta); // error correction

  if(u<MinVPower)
  {
    // Power Error
    robot.Stop();
    myservo.detach();
    while(1)
    {
      digitalWrite(PIN_REFLEX_INDICATOR, 1);
      Beep(1);
      
      PSERIAL.print("***");
      PSERIAL.print(cu);
      PSERIAL.print(" / ");
      PSERIAL.print(u);
      PSERIAL.print(" / ");
      PSERIAL.println(analogRead(PIN_VPOWER));
      
      digitalWrite(PIN_REFLEX_INDICATOR, 0);
    }
  }
  
  // DEBUG
  /*
  PSERIAL.print("***");
  PSERIAL.print(cu);
  PSERIAL.print(" / ");
  PSERIAL.print(u);
  PSERIAL.print(" / ");
  PSERIAL.println(analogRead(PIN_VPOWER));
  */
}

#endif
