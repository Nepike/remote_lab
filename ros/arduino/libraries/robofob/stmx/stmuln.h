/*
 * Управление шаговым двигателем

 *  Сервер управления шаговым двигателем BYJ48
 *  Автор кода: Mohannad Rawashdeh
 *  Детали на русском языке: /arduino-shagovii-motor-28-BYJ48-draiver-ULN2003
 *  Англоязычный вариант: http://www.instructables.com/member/Mohannad+Rawashdeh/ 28/9/2013

 * 01.05.2020
 * LP 10.05.2020
 */

#ifndef _STMULN_H_

#define _STMULN_H_

#include <Arduino.h>

const unsigned int _MIN_DTMI = 1000; //1000;
const unsigned int _MAX_DTMI = 5000;

class TStepMULN2003
{ 
  public:

    int _DTMI;
    
    TStepMULN2003(int _p1, int _p2, int _p3, int _p4)
    {
      pin1 = _p1;
      pin2 = _p2;
      pin3 = _p3;
      pin4 = _p4;

      _steps = 0;
      _direction = 0;
      _DTMI = _MAX_DTMI;
      _is_stopped = 1;
    }
    void Attach(void)
    {
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      pinMode(pin3, OUTPUT);
      pinMode(pin4, OUTPUT);
      Stop();
    }

    void Stop(void)
    {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
      digitalWrite(pin3, LOW);
      digitalWrite(pin4, LOW);
      _is_stopped = 1;
    }
  
    void SetSpeed(int sp)
    {  
      if(sp==0)
      {
        Stop();
        return;
      }
      _is_stopped = 0;
      _direction = (sp>0);
      if(sp>100) sp = 100;
      if(sp<-100) sp = -100;
      if(sp<0) sp = -sp;
      _DTMI = map(100-sp, 0, 100, _MIN_DTMI, _MAX_DTMI);
    }
  
    void Eval(void)
    {
      static unsigned long pred_t = 0;
      unsigned long ct = micros();
      if(ct - pred_t >= _DTMI)
      {
        _step(1);
        pred_t = ct;
      }
    }

    void _step(int);
  
  private:
    int _steps;
    int _direction;
    int pin1, pin2, pin3, pin4;
    int _is_stopped;

    void _set_direction(void)
    {
      if(_direction) _steps++; else _steps--; 
      if(_steps>7) _steps=0;
      if(_steps<0) _steps=7;
    }  
};

#endif
