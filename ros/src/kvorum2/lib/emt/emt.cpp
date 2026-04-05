/*
  08.07.2013
  LP 23.07.2013
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "emt.h"

//----------------------------------------------------------
// Потребности
TVAL param::need_food_base = 40;
TVAL param::need_comfort_base = 40;
TVAL param::need_save_base = 60;

// Такт времени, с TIME_STEP
// Используется для иимтации чувства голода
int param::TIME_STEP = 0;

float param::coeff_em = 0;
float param::coeff_fb = 0;

// Период записи в log-файл, мс
int param::LogFileTMs = 1000;

char param::LogFileName[128] = "";

// Вентили
TVAL
  gate_food,
  gate_obstacle,
  gate_danger;

const char *PROCNAMES[] = {
  "S.Food",   // Поиск пищи
  "Eat",      // Поедание пищи
  "Escape",   // Убегание от препятствия
  "S.Shad",   // Поиск тени
  "Sleep",    // Сон
  "Walk",     // Свободное блуждание
  "MvObst",   // Движение к препятствию
  "NONE",
  // Примитивы
  "PROC_ONE_STEP",
  "PROC_REFLEX",
  "PROC_STOP",
  "PROC_GOFWD",
  "PROC_GOBACK",
  "PROC_GOLEFT",
  "PROC_GORIGHT",
  "PROC_STEPFWD",
  "PROC_STEPBACK",
  "PROC_STEPLEFT",
  "PROC_STEPRIGHT",
  "PROC_BEEP"
};

TVAL etPROC[PROC_NUM];
TVAL etEM[PROC_NUM];
TVAL etFACT[PROC_NUM];
TVAL etOUT[PROC_NUM];

const char *ProcName2Str(int pn)
{
  if(pn<0 || pn>=PROC_LAST) return "???";
  return PROCNAMES[pn];
}

//----------------------------------------------------------
// Ring buffer
//----------------------------------------------------------

BYTE RBLEN = MAX_RB_SIZE/2;

void TRingBuffer::Init(BYTE sz)
{
  size = sz;
  memset(buf,0,sz*sizeof(TVAL));
  cp = 0;
}

void TRingBuffer::SetBSize(BYTE sz)
{
  if(sz<1) sz = 1;
  if(sz>=MAX_RB_SIZE) sz = MAX_RB_SIZE-1;
  size = sz;
}

void TRingBuffer::Add(TVAL val)
{
  if(val>MAX_SIGN) val = MAX_SIGN;
  if(val<-MAX_SIGN) val = -MAX_SIGN;
  buf[cp] = val;
  cp++;
  if(cp>=size) cp = 0;
}

TVAL TRingBuffer::Getavr(void)
{
  float s = 0;
  TVAL avr;
  BYTE i;
  for(i=0;i<size;i++)
    s += buf[i];
  avr = (TVAL)(s/size);
  return avr;
}

TVAL TRingBuffer::Put(TVAL val)
{
  TVAL res;
  Add(val);
  res = Getavr();
  return res;
}

TVAL TRingBuffer::Put_Norm(TVAL val)
{
  TVAL res;
  if(val>MAX_SIGN) val = MAX_SIGN;
  if(val<0) val = 0;
  res = Put(val);
  return res;
}

void vnorm(int &n)
{
  if(n>MAX_SIGN) n = MAX_SIGN;
  if(n<0) n = 0;
}

//----------------------------------------------------------

TRingBuffer
  rbGATE_FOOD,
  rbGATE_OBSTACLE,
  rbGATE_DANGER;

TRingBuffer
  rbPROC[PROC_NUM];

TRingBuffer
  rbOUT[PROC_NUM];

void InitRB(void)
{
  BYTE i;
  rbGATE_FOOD.Init(RBLEN);
  rbGATE_OBSTACLE.Init(RBLEN);
  rbGATE_DANGER.Init(RBLEN);
  for(i=0;i<PROC_NUM;i++)
  {
    rbPROC[i].Init(RBLEN);
    rbOUT[i].Init(RBLEN);
  }
  memset(etEM,0,sizeof(etEM));
}

void RBResize(void)
{
  if(RBLEN<1) RBLEN = 1;
  if(RBLEN>=MAX_RB_SIZE)
    RBLEN = MAX_RB_SIZE-1;

  rbGATE_FOOD.SetBSize(RBLEN);
  rbGATE_OBSTACLE.SetBSize(RBLEN);
  rbGATE_DANGER.SetBSize(RBLEN);

  for(BYTE i=0;i<PROC_NUM;i++)
  {
    rbPROC[i].Init(RBLEN);
    rbOUT[i].Init(RBLEN);
  }
}

//----------------------------------------------------------

/// Текущее время в секундах
unsigned long emt::GetTimeSec(void)
{
  long t = time(NULL);
  return t;
}

/// Текущее время в миллисекундах
unsigned long emt::GetTimeMSec(void)
{
/*
  long t = time(NULL);
  return t;
*/
  struct timeval tv;
  gettimeofday(&tv, NULL);

  unsigned long long ms =
  (unsigned long long)(tv.tv_sec) * 1000 +
  (unsigned long long)(tv.tv_usec) / 1000;
  
  return ms;
}


/// Текущее время в децисекундах (0.1 с)
long emt::GetTimeDSec(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  unsigned long ds =
  (unsigned long long)(tv.tv_sec) * 10 +
  (unsigned long long)(tv.tv_usec) / 100000;

  return ds;
}

//----------------------------------------------------------

void emt::ShowSensorsNeeds(WINDOW *w, int y, msg_kvorum::senneed &sn)
{
#define ATTR 0
  erl::printfPos(w, y++, 1, ATTR, "Sns: Food    Danger  Hungry  Excit  Inhib  Refl  Obst");
  erl::printfPos(w, y++, 1, ATTR, "     %3d     %3d     %3d     %3d    %3d    %3d   %3d",
         sn.sen_food, sn.sen_danger, sn.sen_hungry,
         sn.sen_excit, sn.sen_inhibit,
         sn.sen_reflex, sn.sen_obstacle);
  y++;
  erl::printfPos(w, y++, 1, ATTR, "Nds: Food    Comfort Save");
  erl::printfPos(w, y++, 1, ATTR, "     %3d     %3d     %3d", 
    sn.need_food, sn.need_comfort, sn.need_save);
  wrefresh(w);
}