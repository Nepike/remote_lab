/**
  
  Version 1.03
  Date 09.01.2020
  LP 16.04.2021
*/

#ifndef _FISH_FSM2_
#define _FISH_FSM2_

// Состояния
enum { qS, qA1, qM12, qA2, qM21 };
enum {cond_A1_A1, cond_A1_M12, cond_A1_M21, cond_M12_M12, cond_M12_A2, cond_A2_A2,
      cond_A2_M21, cond_M21_M21, cond_M21_A1, cond_M21_S};
enum {proc_A1_A1, proc_A1_M12, proc_A1_M21, proc_M12_M12, proc_M12_A2, proc_A2_A2,
      proc_A2_M21, proc_M21_M21, proc_M21_A1, proc_M21_S};

// Задача в виде КА (автомат Мили)
class TTask
{  
  public:

    TTask(int _A1, int _A2, int _Sp12, int _Sp21, int _T1, int _T2)
    {
      SetProg(_A1, _A2, _Sp12, _Sp21, _T1, _T2);
      cont_regime = 1;
      curr_proc = -1;
    }
    void Set(int _a) 
    { 
      a = _a; 
      Stop(); 
    }
    void Start(void) 
    {
      cont_regime = 1; 
      q = qA1;
      t = 0;
      //a = A1;
    }
    void StartOnce(void) { Start(); cont_regime = 0; }
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

    int q;    // Текущее состояние
    int a;    // Текущий угол
    int t;    // Текущий счетчик
    unsigned char cont_regime; // Флаг непрерывного выполнения задания
    signed char curr_proc;     // Текущая выполняемая процедура
};

int AStep(class TTask *tsk);

//----------------------------------------------------------
// Описание поведения сервопривода
//----------------------------------------------------------
class TServo
{
  public:
    TServo(int _id, 
      void _set_speed_funcPtr(int id, int speed), 
      void _set_ang_funcPtr(int id, int ang),
      int a_min, int a_max)       
    {
      srv_set_speed_func = _set_speed_funcPtr;
      srv_set_ang_func = _set_ang_funcPtr;
      id = _id;
      mspeed = 0;
      task = 0; // sic NULL
      ANGLE_MIN = a_min;
      ANGLE_MAX = a_max;
      pred_ang = -1;
    }
    void Init()
    {
      srv_set_speed_func(id, mspeed);
      Reset();
    }
    void Reset()
    {
      asrv = (ANGLE_MAX+ANGLE_MIN)/2;
      srv_set_ang_func(id, asrv);
    }
    void SetAng(int _ang) 
    { 
      asrv = _ang;
      if(task) task->a = asrv;
    }
    int Exec(void) 
    { 
      if(!task) return -1;
      return _set_ang(asrv);
    }
    void MakeStep(void) 
    {
      if(!task) return;
      int ca = AStep(task);
      if(ca>=0) asrv = ca;
    }
    void SetTask(class TTask *t)
    {
      task = t;
      if(task) task->Set(asrv);
    }
    int StartTask(void) 
    { 
      if(!task) return 0;
      task->a = asrv;
      task->Start();
      return 1;
    }
    void StartTaskOnce(void) 
    { 
      if(!task) return;
      task->StartOnce(); 
    }
    void StopTask(void) 
    { 
      if(!task) return;
      task->Stop(); 
    }
    void get_stat(int &sa, int &ta, int &tt, int &tq, int &tproc) 
    { 
      sa = asrv; 
      ta = task?task->a:0;
      tt = task?task->t:0;
      tq = task?task->q:0;
      tproc = task?task->curr_proc:-1;
    }
    int GetA(void) { return asrv; }

  private:
    class TTask *task;
    int id;
    int mspeed;  // Выбирать в диапазоне от 50 до 250. 0 - максимальное значение
    int asrv;    // Текущий угол сервомашинки
    int pred_ang;

    void (*srv_set_speed_func)(int id, int speed);
    void (*srv_set_ang_func)(int id, int a);

    int ANGLE_MIN;
    int ANGLE_MAX;        
    int _set_ang(int _ang) 
    { 
      asrv = _ang;
      if(task) task->a = asrv;
      if (asrv!=pred_ang)
      {
        srv_set_ang_func(id, asrv);
        pred_ang = asrv;
        return 1;
      }
      return 0;
    }
};

#endif
