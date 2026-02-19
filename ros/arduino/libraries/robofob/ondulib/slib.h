/*
  V 1.1

  08.07.2023
  LP 12.07.2023
*/

#ifndef _SLIB_H_
#define _SLIB_H_

namespace slib
{
char sline[80];

// Чтение кнопки в разных режимах
#define DTIME 400
int ReadButton(int pin, bool wait = false, int del_time = DTIME)
{
  int n = !digitalRead(pin);
  if(!n) return 0;
  if(wait)
  {
    while(!digitalRead(pin)) delay(10);
  }
  else
  {  
    unsigned long t = millis();
    //while((millis()-t)<DTIME && digitalRead(pin)==0);
  }
  return 1;    
}  

// Пауза на tm миллисекунд
void msPause(long int tm)
{
  long int t0 = millis();  
  while(millis()-t0<tm);
}

void Beep(int8_t pin_beep, byte n)
{
  for(byte i=0;i<n;i++)
  {
    digitalWrite(pin_beep, HIGH);
    delay(200);
    digitalWrite(pin_beep, LOW);
    delay(200);
  }
}

};

#endif
