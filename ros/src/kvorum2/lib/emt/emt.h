/*
 * Temperament tools and elements
 * 08.07.2013
 * LP 02.03.2016
*/

#ifndef _EMTLIB_H_
#define _EMTLIB_H_

#include <curses.h>
#include <msg_kvorum/senneed.h>
#include "../../../lib/erlib.h"
#include "../../../lib/servlib.h"

#ifndef BYTE
typedef unsigned char BYTE;
#endif

#ifndef TVAL
typedef short int TVAL;
#endif

inline TVAL min2(TVAL a, TVAL b) { return a<b?a:b; }
inline TVAL max2(TVAL a, TVAL b) { return a>b?a:b; }
inline TVAL min3(TVAL a, TVAL b, TVAL c) { return min2(min2(a,b), c); }

namespace param
{
// Потребности
extern TVAL 
  need_food_base,
  need_comfort_base,
  need_save_base;
// Такт времени, мс TIME_STEP
extern int TIME_STEP;

extern float 
  coeff_em,
  coeff_fb;

// Период записи в log-файл, мс
extern int LogFileTMs;

extern char LogFileName[];

};

// Вентили
extern TVAL
  gate_food,
  gate_obstacle,
  gate_danger;


// Процедуры
enum
{ PROC_SEARCH_FOOD,       // Поиск пищи
  PROC_EAT,               // Поедание пищи
  PROC_ESCAPE,            // Убегание от препятствия
  PROC_SEARCH_SHADOW,     // Поиск тени
  PROC_SLEEP,             // Сон
  PROC_WALK,              // Свободное блуждание
  PROC_MOVE_TO_OBSTACLE,  // Движение к препятствию
  PROC_NONE,
  PROC_ONE_STEP,
  PROC_REFLEX,
  // Остальные подпрограммы являются примитивами
  PROC_STOP,
  PROC_GOFWD,
  PROC_GOBACK,
  PROC_GOLEFT,
  PROC_GORIGHT,
  PROC_STEPFWD,
  PROC_STEPBACK,
  PROC_STEPLEFT,
  PROC_STEPRIGHT,
  PROC_BEEP,
  PROC_LAST
};

const char *ProcName2Str(int pn);

#define PROC_NUM PROC_NONE

extern TVAL etPROC[PROC_NUM];
extern TVAL etEM[PROC_NUM];
extern TVAL etFACT[PROC_NUM];
extern TVAL etOUT[PROC_NUM];

//----------------------------------------------------------
// Parameters
//----------------------------------------------------------

#define MAX_SIGN 100

//----------------------------------------------------------
// Ring buffer
//----------------------------------------------------------

#define MAX_RB_SIZE 20

extern BYTE RBLEN;

struct TRingBuffer
{
  BYTE size;
  TVAL buf[MAX_RB_SIZE];
  BYTE cp;
  //---
  void Add(TVAL val);
  TVAL Getavr(void);
  TVAL Put(TVAL val);
  TVAL Put_Norm(TVAL val);
  void SetBSize(BYTE sz);
  void Init(BYTE sz);
};

//----------------------------------------------------------

extern TRingBuffer
  rbGATE_FOOD,
  rbGATE_OBSTACLE,
  rbGATE_DANGER;

extern TRingBuffer
  rbPROC[PROC_NUM];

extern TRingBuffer
  rbOUT[PROC_NUM];

void InitRB(void);

void RBResize(void);

void vnorm(int &n);

namespace emt
{
/// Текущее время в миллисекундах
unsigned long GetTimeMSec(void);

/// Текущее время в децисекундах (0.1 с)
long GetTimeDSec(void);

/// Текущее время в секундах
unsigned long GetTimeSec(void);

void ShowSensorsNeeds(WINDOW *w, int y, msg_kvorum::senneed &sn);
};

#endif
