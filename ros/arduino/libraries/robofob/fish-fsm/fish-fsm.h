/**
  
  Version 1.03
  Date 09.01.2020
  LP 16.04.2021
*/


#ifndef _FISH_FSM_
#define _FISH_FSM_

#include "maestro2.h"


extern void srv_putchar(unsigned char c);
extern unsigned char srv_getchar(void);

extern char SOUT[];

#define ID_DEV 12 // Идентификатор устройства Pololu Maestro. По умолчанию = 12

extern TMServo mServo;
extern const int ANGLE_MIN;
extern const int ANGLE_MAX;

// Состояния
enum { qS, qA1, qA2, qM12, qM21 };

class TTask
{  
  public:

    TTask(int _A1, int _A2, int _Sp12, int _Sp21, int _T1, int _T2)
    {
      SetProg(_A1, _A2, _Sp12, _Sp21, _T1, _T2);
    }
    void Set(int _a) 
    { 
      a = _a; 
      Stop(); 
    }
    void Start(void) { q = qM12; }
    void Stop(void) { q = qS; }

    void SetProg(int _A1, int _A2, int _Sp12, int _Sp21, int _T1, int _T2)
    {
      A1 = _A1;
      A2 = _A2;
      Speed12 = _Sp12;
      Speed21 = _Sp21;
      T1 = _T1;
      T2 = _T2;
      a = 0;
      t = 0;
      q = qS;
    }
    
    int A1, A2;           // Углы
    int Speed12, Speed21; // Скорости
    int T1, T2;           // Задержки в крайних позициях (время паузы в тиках)

    int q;  // Текущее состояние
    int a;  // Текущий угол
    int t;  // Текущий счетчик
};

int AStep(class TTask *tsk);

//----------------------------------------------------------
// Описание поведения сервопривода
//----------------------------------------------------------
class TServo
{
  public:
    TServo(int _id)
    {
      id = _id;
      mspeed = 0;
      task = 0; // sic NULL
    }
    void Init()
    {
      mServo.SetSpeed(ID_DEV, id, mspeed);
      Reset();
    }
    void Reset()
    {
      a = (ANGLE_MAX+ANGLE_MIN)/2;
      SetAng(a);
    }
    void SetAng(int _ang) 
    { 
      static int pred_ang = -1;
      a = _ang;
      //if (a!=pred_ang)
      {
        mServo.SetAng(ID_DEV, id, a);
        pred_ang = a;
      }
    }
    void Exec(void) 
    { 
      if(!task) return;
      if(task->q == qM12 || task->q == qM21)
        SetAng(a); 
    }
    void MakeStep(void) 
    {
      if(!task) return;
      int ca = AStep(task);
      // Если убрать следующую строку, то программа перестанет работать
      //sprintf(SOUT,"%d %d", id, ca);      
      //Serial.println(SOUT);
      if(ca>=0) a = ca;
    }
    void SetTask(class TTask *t)
    {
      task = t;
      if(task) task->Set(a); 
    }
    void StartTask(void) 
    { 
      if(!task) return;
      task->Start(); 
    }
    void StopTask(void) 
    { 
      if(!task) return;
      task->Stop(); 
    }
    
  private:
    class TTask *task;
    int id;
    int mspeed;    // Выбирать в диапазоне от 50 до 250. 0 - максимальное значение
    int a;         // Текущий угол
};

#endif
