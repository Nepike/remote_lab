/**
  
  Version 1.03
  Date 09.01.2020
  LP 16.04.2021
*/

#include "fish-fsm2.h"

//----------------------------------------------------------
// Описание поведения сервопривода
//----------------------------------------------------------
class TRule
{
  public: int q1, q2, cond, proc;
};

const int NR = 10;

TRule rules[NR] = 
{{ qA1, qA1,  cond_A1_A1,  proc_A1_A1 },
 { qA1, qM21, cond_A1_M21, proc_A1_M21 },
 { qA1, qM12, cond_A1_M12, proc_A1_M12 },

 { qM12, qM12, cond_M12_M12, proc_M12_M12 },
 { qM12, qA2,  cond_M12_A2,  proc_M12_A2 },

 { qA2, qA2,  cond_A2_A2,  proc_A2_A2 },
 { qA2, qM21, cond_A2_M21, proc_A2_M21 },

 { qM21, qM21, cond_M21_M21,proc_M21_M21 },
 { qM21, qA1,  cond_M21_A1, proc_M21_A1 },
 { qM21, qS,  cond_M21_S, proc_M21_S }};

int AStep(class TTask *tsk)
{
  int proc = -1;
  if(!tsk) return -4;

  for(int i=0;i<NR;i++)
  {
    if(tsk->q==rules[i].q1)
    {
      // check cond
      int res = 0;
      switch(rules[i].cond)
      {
        case cond_A1_A1:   res = tsk->t < tsk->T1; break;
        case cond_A1_M21:  res = tsk->a > tsk->A2; break;
        case cond_A1_M12:  res = 1; break;

        case cond_M12_M12: res = tsk->a < tsk->A2; break;
        case cond_M12_A2:  res = 1; break;

        case cond_A2_A2:   res = tsk->t < tsk->T2; break;
        case cond_A2_M21:  res = 1; break;

        case cond_M21_M21: res = tsk->a > tsk->A1; break;
        case cond_M21_A1:  res = tsk->cont_regime; break;
        case cond_M21_S:   res = 1; break;

        default: return -1;
      }
      if(res)
      {
        proc = rules[i].proc;
        tsk->q = rules[i].q2;
        break;
      }
    }
  }
  tsk->curr_proc = proc;
  if(proc<0) return -2;

  // exec proc
  switch(proc)
  {
    case proc_A1_A1: 
    case proc_A2_A2: 
      tsk->t++;
      break;
    case proc_A1_M21:
    case proc_A1_M12:
      break;
    case proc_M12_M12:
      tsk->a = tsk->a + tsk->Speed12;
      if (tsk->a > tsk->A2) tsk->a = tsk->A2;
      break;
    case proc_M12_A2:
      tsk->t = 0;
      tsk->a = tsk->A2;
      break;
    case proc_A2_M21:  break;
    case proc_M21_M21: 
      tsk->a = tsk->a - tsk->Speed21;
      if (tsk->a < tsk->A1) tsk->a = tsk->A1;
      break;
    case cond_M21_A1:
    case proc_M21_S:   
      tsk->t = 0;
      tsk->a = tsk->A1;
      break;
    default: return -3;
  }
  return tsk->a;
}
