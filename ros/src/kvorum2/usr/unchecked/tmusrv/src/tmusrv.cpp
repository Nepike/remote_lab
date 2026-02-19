/**
 * \file
 * Эмоции и темперамент робота
 * rcX protocol
 * \author Robofob
 * \version 2.03
 *
 * \date 01.08.2013
 * \date LP 11.06.2016
 *
 * Извне приходит номер процедуры: PROC_SLEEP, PROC_ESCAPE, ...
 * L: 843 852 856 883 896 678 635, 733, 799
 *
 */

#include <sstream>
#include <curses.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <msg_kvorum/proc.h>
#include <msg_kvorum/senneed.h>

#include <msg_ans/ans.h>
#include <msg_rsaction/action.h>

#include "../../../../lib/rcproto/rcproto2.h"
#include "../../../../lib/rcproto/tmucmd.h"
#include "../../../../../arduino/libraries/i2cpro/i2cwctl.h"
#include "../../../../lib/erlib.h"
#include "../../../../lib/servlib.h"
#include "../../../lib/emt/emt.h"
#include "../../../lib/auto/auto.h"
#include "../../../lib/tmurobot/tmurobot.h"

//------------------------------------------------------------------------------

const char *Title = "\nTMU Server 2.03\n";

WINDOW *w_main = NULL;

ros::Publisher chatter_senneed_pub;
ros::Publisher chatter_action_pub;

msg_kvorum::senneed VSenNeed;

// Параметры состояния программы
int ROB_CMD = PROC_NONE;        // Поведенческая процедура

long CurrentTime = 0;
int y_show_last = 0;
TAStack FSMStack; // Стек автоматов

//------------------------------------------------------------------------------
// Автоматы
//------------------------------------------------------------------------------

// Состояния управляющего автомата
enum { qS, qT, q1, q2, q3, q4, q5 };

const char *ROBQNAMES[] = { "qS", "qT", "q1", "q2", "q3", "q4", "q5" };

const char *RobCond2Str(int pn)
{
  if(pn<0) return "???";
  return ROBQNAMES[pn];
}

// Автоматные действия
const char *fProcGoFwd(TAutomaton *a, int getname);
const char *fProcGoLeft(TAutomaton *a, int getname);
const char *fProcGoRight(TAutomaton *a, int getname);
const char *fProcGoBack(TAutomaton *a, int getname);
const char *fProcStop(TAutomaton *a, int getname);
const char *fProcEat(TAutomaton *a, int getname);
const char *fMakeOneStepProc(TAutomaton *a, int getname);
const char *fProcRandomTurn(TAutomaton *a, int getname);

const char *fCheckTcnt(TAutomaton *a, int getname);
const char *fCheckFood(TAutomaton *a, int getname);
const char *fCheckShadow(TAutomaton *a, int getname);
const char *fIsLB(TAutomaton *a, int getname);
const char *fIsRB(TAutomaton *a, int getname);
const char *fIsLRB(TAutomaton *a, int getname);

const char *fCheckObstFwd(TAutomaton *a, int getname);
const char *fCheckObst(TAutomaton *a, int getname);
const char *fCheckObstLeft(TAutomaton *a, int getname);
const char *fCheckObstRight(TAutomaton *a, int getname);
const char *fCheckNoObst(TAutomaton *a, int getname);
const char *fCheckFreeFwd(TAutomaton *a, int getname);
const char *fCheckTcntAndFreeFwd(TAutomaton *a, int getname);
const char *fCheckFreeLeft(TAutomaton *a, int getname);
const char *fCheckTcntAndFreeLeft(TAutomaton *a, int getname);
const char *fCheckFreeRight(TAutomaton *a, int getname);
const char *fCheckTcntAndFreeRight(TAutomaton *a, int getname);

// Поиск пищи
TRule *TTR_SEARCH_FOOD[] =
{
  new TRule(qS, qT, fCheckFood, fProcStop),
  new TRule(qS, q1, NULL,       fProcGoFwd),
  new TRule(q1, qT, fCheckFood, fProcStop),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q2, NULL,       fProcRandomTurn),
  new TRule(q2, qT, fCheckFood, fProcStop),
  new TRule(q2, q2, fCheckTcnt, NULL),
  new TRule(q2, q3, NULL,       fProcGoFwd),
  new TRule(q3, qT, fCheckFood, fProcStop),
  new TRule(q3, q3, fCheckTcnt, NULL),
  new TRule(q3, q1, NULL,       fProcGoFwd),
  NULL
};

// Поиск тени
TRule *TTR_SEARCH_SHADOW[] =
{
  new TRule(qS, qT, fCheckShadow, fProcStop),
  new TRule(qS, q1, NULL,         fProcGoFwd),
  new TRule(q1, qT, fCheckShadow, fProcStop),
  new TRule(q1, q1, fCheckTcnt,   NULL),
  new TRule(q1, q2, NULL,         fProcRandomTurn),
  new TRule(q2, qT, fCheckShadow, fProcStop),
  new TRule(q2, q2, fCheckTcnt,   NULL),
  new TRule(q2, q3, NULL,         fProcGoFwd),
  new TRule(q3, qT, fCheckShadow, fProcStop),
  new TRule(q3, q3, fCheckTcnt,   NULL),
  new TRule(q3, q1, NULL,         fProcGoFwd),
  NULL
};

// Свободное блуждание
TRule *TTR_WALK[] =
{
  new TRule(qS, q1, NULL,       fProcGoFwd),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q2, NULL,       fProcRandomTurn),
  new TRule(q2, q2, fCheckTcnt, NULL),
  new TRule(q2, q3, NULL,       fProcGoFwd),
  new TRule(q3, q3, fCheckTcnt, NULL),
  new TRule(q3, qS, NULL,       fProcGoFwd),
  NULL
};

// Сон
TRule *TTR_SLEEP[] =
{
  new TRule(qS, q1, NULL,       fProcStop),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q2, NULL,       fProcRandomTurn),
  new TRule(q2, q2, fCheckTcnt, NULL),
  new TRule(q2, q3, NULL,       fProcStop),
  new TRule(q3, q3, fCheckTcnt, NULL),
  new TRule(q3, q1, NULL,       NULL),
  NULL
};

// Поедание пищи
TRule *TTR_EAT[] =
{
  new TRule(qS, q1, NULL,       fProcStop),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q2, NULL,       fProcGoLeft),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q2, NULL,       fProcEat),
  new TRule(q2, q2, fCheckTcnt, NULL),
  new TRule(q2, q3, NULL,       fProcEat),
  new TRule(q3, q3, fCheckTcnt, NULL),
  new TRule(q3, q4, NULL,       fProcGoLeft),
  new TRule(q4, q4, fCheckTcnt, NULL),
  new TRule(q4, q1, NULL,       fProcEat),
  NULL
};

// Движение к препятствию
TRule *TTR_MOVE_TO_OBST[] =
{
  new TRule(qS, qT, fCheckObstFwd,   fProcStop),
  new TRule(qS, qS, fCheckObstLeft,  fProcGoLeft),
  new TRule(qS, qS, fCheckObstRight, fProcGoRight),
  new TRule(qS, q1, NULL,            fProcGoFwd),
  
  new TRule(q1, qT, fCheckObstFwd,   fProcStop),
  new TRule(q1, qS, fCheckObst,      NULL),
  new TRule(q1, q1, fCheckTcnt,      NULL),
  new TRule(q1, q2, NULL,            fProcRandomTurn),
  
  new TRule(q2, qT, fCheckObstFwd,   fProcStop),
  new TRule(q2, qS, fCheckObst,      NULL),
  new TRule(q2, q2, fCheckTcnt,      NULL),
  new TRule(q2, q3, NULL,            fProcGoFwd),
  
  new TRule(q3, qT, fCheckObstFwd,   fProcStop),
  new TRule(q3, qS, fCheckObst,      NULL),
  new TRule(q3, q3, fCheckTcnt,      NULL),
  new TRule(q3, q1, NULL,            fProcGoFwd),
  NULL
};

// Убегание от опасности (от препятствия)
TRule *TTR_ESCAPE[] =
{
  new TRule(qS, qT, fCheckNoObst,           fProcStop),
  new TRule(qS, q1, fCheckFreeFwd,          fProcGoBack),
  new TRule(qS, q3, fCheckFreeLeft,         fProcGoLeft),
  new TRule(qS, q4, fCheckFreeRight,        fProcGoRight),
  new TRule(qS, qS, NULL,                   fProcGoBack),

  new TRule(q1, q1, fCheckTcntAndFreeFwd,   NULL),
  new TRule(q1, q2, NULL,                   fProcGoLeft),

  new TRule(q2, q2, fCheckTcnt,             NULL),
  new TRule(q2, qS, NULL,                   fProcStop),

  new TRule(q3, q3, fCheckTcntAndFreeLeft,  NULL),
  new TRule(q3, qS, NULL,                   fProcStop),

  new TRule(q4, q4, fCheckTcntAndFreeRight, NULL),
  new TRule(q4, qS, NULL,                   fProcStop),
  NULL
};

// Рефлекс
TRule *TTR_REFLEX[] =
{
  new TRule(qS, q1, fIsLRB,     fProcGoBack),
  new TRule(qS, q1, fIsLB,      fProcGoBack),
  new TRule(qS, q2, fIsRB,      fProcGoBack),
  new TRule(qS, qT,  NULL,      fProcStop),
  new TRule(q1, q1, fCheckTcnt, NULL),
  new TRule(q1, q3, NULL,       fProcGoRight),
  new TRule(q2, q2, fCheckTcnt, NULL),
  new TRule(q2, q3, NULL,       fProcGoLeft),
  new TRule(q3, q3, fCheckTcnt, NULL),
  new TRule(q3, qT, NULL,       fProcStop),
  NULL
};

// Разовое действие
TRule *TTR_ONE_STEP[] =
{
  new TRule(qS, q1, NULL, fMakeOneStepProc),
  new TRule(q1, q1, NULL, NULL),
  NULL
};

TAutomaton *Automatons[] =
{
  new TAutomaton(PROC_SEARCH_FOOD,      "A_SEARCH_FOOD",   qS, qT, 10),
  new TAutomaton(PROC_SEARCH_SHADOW,    "A_SEARCH_SHADOW", qS, qT, 10),
  new TAutomaton(PROC_REFLEX,           "A_REFLEX",        qS, qT, 5),
  new TAutomaton(PROC_ONE_STEP,         "A_ONE_STEP",      qS, qT, 0),
  new TAutomaton(PROC_WALK,             "A_WALK",          qS, qT, 10),
  new TAutomaton(PROC_SLEEP,            "A_SLEEP",         qS, qT, 20),
  new TAutomaton(PROC_EAT,              "A_EAT",           qS, qT, 5),
  new TAutomaton(PROC_ESCAPE,           "A_ESCAPE",        qS, qT, 5),
  new TAutomaton(PROC_MOVE_TO_OBSTACLE, "A_MOVE_TO_OBST",  qS, qT, 10),
  NULL
};

TAutomaton *AFind(int id, int fatalerr = 1)
{
  for(int i=0; Automatons[i]; i++)
    if(Automatons[i]->GetId()==id) return Automatons[i];
  if(fatalerr)
    error("\nFSM id %d '%s' not found", id, ProcName2Str(id));
  return NULL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

TRobot Robot(1);

int
  // Для определения свободного пространста
  // 0 - свободно, +1 - свобода слева, -1 - свобода справа
  FreeWayDirection = 0,
  FreeWayCPos = 0,      // Центр тяжести искомого
  FreeWayWeight,        // Вес искомого

  // Для определения занятого пространста
  // 0 - свободно, +1 - препятствие слева, -1 - препятствие справа
  ObstacleDirection = 0,
  ObstacleCPos = 0,      // Центр тяжести искомого
  ObstacleWeight;        // Вес искомого

//------------------------------------------------------------------------------
// Выбор автомата для исполнения команды procnum
//------------------------------------------------------------------------------
TAutomaton *SelectAutomaton(int procnum)
{
  if(procnum==PROC_NONE) return NULL;

  ROB_CMD = procnum; // Сохраняем для функции TRobot::Make
  TAutomaton *a = AFind(procnum, 0);
  if(!a) // Не нашли. Пробуем использовать автомат для разового действия
    a = AFind(PROC_ONE_STEP);
  return a;
}

/// Прием командного сообщения
void GetProcMsg(const msg_kvorum::proc &msg)
{
  int procnum = msg.proc;
  // Выбор автомата для исполнения команды currProcedure
  TAutomaton *ca = SelectAutomaton(procnum);
  if(ca)
  {
    #define ATTR 0
    erl::printfPos(w_main, y_show_last+1, 1, ATTR, "-- [%ld] Load FSM %-15s", time(NULL), ca->GetName());
    ca->Reset();
    FSMStack.Clear();
    FSMStack.Push(ca);
  }
}

//------------------------------------------------------------------------------
//
// Прием сообщения
//
//------------------------------------------------------------------------------
void GetAnsMsg(const msg_ans::ans &msg)
{
  Robot.AcceptMessage(msg);
}

/// Обработка рецепторов. Формирование значений сенсоров и потребностей
void EvaluateSensorsInfo(void)
{
  static long pred_ctime = 0;
  TVAL rspot = Robot.ReverseSpotSens?(TVAL)(255-(TVAL)Robot.REC_SPOT()):(TVAL)Robot.REC_SPOT();

  VSenNeed.sen_food = Robot.rbSENSORS[SEN_FOOD].Put(rspot);
  VSenNeed.sen_danger = Robot.rbSENSORS[SEN_DANGER].Put((TVAL)Robot.REC_LIGHT());

  // Определяем свободный проход (и занятый)
  FreeWayDirection = Robot.FindLocObj(&FreeWayCPos, &FreeWayWeight);
  ObstacleDirection = Robot.FindLocObj(&ObstacleCPos, &ObstacleWeight, 1);

  VSenNeed.sen_obstacle = Robot.rbSENSORS[SEN_OBSTACLE].Put((TVAL)(FreeWayDirection==0?0:ObstacleWeight));

  // Рефлексы
  VSenNeed.sen_reflex = REFLEX_NONE;
  int rleft = (Robot.REC_SHARP_FL()>Robot.IR_LIM);
  int rright = (Robot.REC_SHARP_FR()>Robot.IR_LIM);
  if(rleft && rright)
    VSenNeed.sen_reflex = REFLEX_ALL;
  else
  if(rleft)
    VSenNeed.sen_reflex = REFLEX_LEFT;
  else
  if(rright)
    VSenNeed.sen_reflex = REFLEX_RIGHT;

  // Время и голод (время - в 0.1 сек.)
  long ctime = emt::GetTimeDSec();
  if(ctime-pred_ctime>param::TIME_STEP)
  {
    pred_ctime = ctime;
    VSenNeed.sen_hungry++;
    if(VSenNeed.sen_hungry>MAX_SIGN) VSenNeed.sen_hungry = MAX_SIGN;
  }
  // Потребность в пище
  //??? VSenNeed.need_food = (TVAL)(param::need_food_base + VSenNeed.sen_hungry);
  //vnorm(VSenNeed.need_food);

  // Возбуждение и торможение
  VSenNeed.sen_excit = (BYTE)((int)Robot.REC_EXCIT()*100/255);
  VSenNeed.sen_inhibit = (BYTE)((int)Robot.REC_INHIBIT()*100/255);
  if(VSenNeed.sen_inhibit<1) VSenNeed.sen_inhibit = 1;

  // Размер буфера = сила торможения
  static BYTE pred_sen_inhibit = 0;
  if(VSenNeed.sen_inhibit != pred_sen_inhibit)
  {
    pred_sen_inhibit = VSenNeed.sen_inhibit;

    rbGATE_FOOD.SetBSize(VSenNeed.sen_inhibit);
    rbGATE_OBSTACLE.SetBSize(VSenNeed.sen_inhibit);
    rbGATE_DANGER.SetBSize(VSenNeed.sen_inhibit);
    for(int i=0;i<PROC_NUM;i++)
    {
      rbPROC[i].SetBSize(VSenNeed.sen_inhibit);
      rbOUT[i].SetBSize(VSenNeed.sen_inhibit);
    }
  }
  // Публикуем
  chatter_senneed_pub.publish(VSenNeed);
}

void ReadNeeds(char *inifile)
{
  l_string s;
  FILE *f=fopen(inifile,"r");
  if(!f)
    error("open ini file %s error", inifile);

  // * Такт времени, с TIME_STEP
  param::TIME_STEP = ReadInt(f);

  // * needs base
  if(!SkipRemarkLine(f, s))
    error("format error 4");
  int a1, a2, a3;
  sscanf(s,"%d %d %d", &a1, &a2, &a3);
  param::need_food_base = (TVAL)a1;
  param::need_comfort_base = (TVAL)a2;
  param::need_save_base = (TVAL)a3;

  VSenNeed.need_food = param::need_food_base;
  VSenNeed.need_comfort = param::need_comfort_base;
  VSenNeed.need_save = param::need_save_base;

  fclose(f);
}

//------------------------------------------------------------------------------

void ShowRobotStatus(TAutomaton *a)
{
  #define ATTR 0
  int y = 3;
  Robot.ShowReceptors(w_main,1, y);
  y+=3;

  // TSOP RC5
  erl::printfPos(w_main, y, 1,  ATTR, "TSOP (%d): ", Robot.TSOPRC5.size());
  for(int i=0;i<Robot.TSOPRC5.size();i++)
    wprintw(w_main, "%04X ", Robot.TSOPRC5[i]);

  y+=1;
  emt::ShowSensorsNeeds(w_main, y, VSenNeed);
  y+=5;
  // Локатор
  wmove(w_main, y, 1);
  int nloc = Robot.Locator.size();
  for(int i=0;i<nloc;i++)
    wprintw(w_main, "%c", Robot.Locator[i]?'*':'.');
  wprintw(w_main, " %2d >> (d:%2d/%2d p:%2d/%2d w:%2d/%2d)  ",
          nloc, FreeWayDirection, ObstacleDirection, FreeWayCPos, ObstacleCPos, FreeWayWeight, ObstacleWeight);
  y+=2;
  // Текущее состояние программы
  erl::printfPos(w_main, y, 1, ATTR, "-- Time: %-4d  [A=%-15s P=%-12s]  Q:=%-6s %3d ReflexRegime: %d ",
                 CurrentTime, a->GetName(), ProcName2Str(Robot.LAST_ROB_CMD), RobCond2Str(a->CurrState()), a->tcnt, VSenNeed.sen_reflex);

  y+=2;
  // Показываем регистры
  erl::printfPos(w_main, y, 1, ATTR, "REG  HEX  DEC  (%d)", Robot.Registers.size());
  for(int i=0;i<Robot.Registers.size();i++)
  {
    y++;
    erl::printfPos(w_main, y, 1, ATTR, "R%02d: %04X %3d", i, Robot.Registers[i], Robot.Registers[i]);
  }
  y_show_last = y;
  wrefresh(w_main);
}

//------------------------------------------------------------------------------
// Автоматные процедуры
//------------------------------------------------------------------------------
#define SIG_0 (char*)0
#define SIG_1 (char*)1

#define fPROLOG(NAME) if(a==NULL) AError(a, (NAME)); if(getname) return (NAME);

const char *fProcGoFwd(TAutomaton *a, int getname)
{
  fPROLOG("fProcGoFwd")
  Robot.Make(PROC_GOFWD);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcGoLeft(TAutomaton *a, int getname)
{
  fPROLOG("fProcGoLeft");
  Robot.Make(PROC_GOLEFT);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcRandomTurn(TAutomaton *a, int getname)
{
  fPROLOG("fProcRandomTurn");
  if(rand()%2==0)
    Robot.Make(PROC_GOLEFT);
  else
    Robot.Make(PROC_GORIGHT);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcGoRight(TAutomaton *a, int getname)
{
  fPROLOG("fProcGoRight");
  Robot.Make(PROC_GORIGHT);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcGoBack(TAutomaton *a, int getname)
{
  fPROLOG("fProcGoBack");
  Robot.Make(PROC_GOBACK);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcStop(TAutomaton *a, int getname)
{
  fPROLOG("fProcGoBack");
  Robot.Make(PROC_STOP);
  a->tcnt = 0;
  return SIG_1;
}

const char *fProcEat(TAutomaton *a, int getname)
// Поедание пищи
{
  fPROLOG("fProcEat");
  int food = (VSenNeed.sen_food!=0);
  if(food)
  {
    VSenNeed.sen_hungry = 0; // -=10
    Robot.Make(PROC_EAT);
  }
  a->tcnt = 0;
  return SIG_1;
}

//------------------------------------------------------------------------------
// Автоматные предикаты
//------------------------------------------------------------------------------

const char *fCheckTcnt(TAutomaton *a, int getname)
{
  fPROLOG("fCheckTcnt");
  int res = a->TcntCheck();
  return res? SIG_1 : SIG_0;
}

const char *fCheckFood(TAutomaton *a, int getname)
{
  fPROLOG("fCheckFood");
  int food = (VSenNeed.sen_food!=0);
  return (food) ? SIG_1 : SIG_0;
}

const char *fCheckShadow(TAutomaton *a, int getname)
{
  fPROLOG("fCheckShadow");
  int shadow = (VSenNeed.sen_danger==0);
  return (shadow) ? SIG_1 : SIG_0;
}

const char *fIsLB(TAutomaton *a, int getname)
{
  fPROLOG("fIsLB");
  int res = VSenNeed.sen_reflex == REFLEX_LEFT;
  return res? SIG_1 : SIG_0;
}

const char *fIsRB(TAutomaton *a, int getname)
{
  fPROLOG("fIsRB");
  int res = VSenNeed.sen_reflex == REFLEX_RIGHT;
  return res? SIG_1 : SIG_0;
}

const char *fIsLRB(TAutomaton *a, int getname)
{
  fPROLOG("fIsLRB");
  int res = VSenNeed.sen_reflex == REFLEX_ALL;
  return res? SIG_1 : SIG_0;
}

const char *fMakeOneStepProc(TAutomaton *a, int getname)
{
  fPROLOG("fMakeOneStepProc");
  Robot.Make(ROB_CMD);
  ROB_CMD = PROC_NONE;
  return SIG_1;
}

//----------------------------------------------------------

const char *fCheckNoObst(TAutomaton *a, int getname)
{
  fPROLOG("fCheckNoObst");
  int res = (Robot.Locator.size() == FreeWayWeight);
  return res? SIG_1 : SIG_0;
}

const char *fCheckObst(TAutomaton *a, int getname)
{
  fPROLOG("fCheckObst");
  int res = (ObstacleWeight>0);
  return res? SIG_1 : SIG_0;
}

const char *fCheckObstFwd(TAutomaton *a, int getname)
{
  fPROLOG("fCheckObstFwd");
  int res = (ObstacleDirection == TMU_LOC_DIR_FWD);
  return res? SIG_1 : SIG_0;
}

const char *fCheckObstLeft(TAutomaton *a, int getname)
{
  fPROLOG("fCheckObstLeft");
  int res = (ObstacleDirection == TMU_LOC_DIR_LEFT);
  return res? SIG_1 : SIG_0;
}

const char *fCheckObstRight(TAutomaton *a, int getname)
{
  fPROLOG("fCheckObstRight");
  int res = (ObstacleDirection == TMU_LOC_DIR_RIGHT);
  return res? SIG_1 : SIG_0;
}

const char *fCheckFreeFwd(TAutomaton *a, int getname)
{
  fPROLOG("fCheckFreeFwd");
  int res = (FreeWayDirection == TMU_LOC_DIR_FWD);
  return res? SIG_1 : SIG_0;
}

const char *fCheckTcntAndFreeFwd(TAutomaton *a, int getname)
{
  int res = a->TcntCheck() && (FreeWayDirection == TMU_LOC_DIR_FWD);
  fPROLOG("fCheckTcntAndFreeFwd");
  return res? SIG_1 : SIG_0;
}

const char *fCheckFreeLeft(TAutomaton *a, int getname)
{
  fPROLOG("fCheckFreeLeft");
  int res = (FreeWayDirection == TMU_LOC_DIR_LEFT);
  return res? SIG_1 : SIG_0;
}

const char *fCheckTcntAndFreeLeft(TAutomaton *a, int getname)
{
  fPROLOG("fCheckTcntAndFreeLeft");
  int res = a->TcntCheck() && (FreeWayDirection == TMU_LOC_DIR_LEFT);
  return res? SIG_1 : SIG_0;
}

const char *fCheckFreeRight(TAutomaton *a, int getname)
{
  fPROLOG("fCheckFreeRight");
  int res = (FreeWayDirection == TMU_LOC_DIR_RIGHT);
  return res? SIG_1 : SIG_0;
}

const char *fCheckTcntAndFreeRight(TAutomaton *a, int getname)
{
  fPROLOG("fCheckFreeRight");
  int res = a->TcntCheck() && (FreeWayDirection == TMU_LOC_DIR_RIGHT);
  return res? SIG_1 : SIG_0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  //--------------------------------------------------------
  // Автоматы
  //--------------------------------------------------------
  AFind(PROC_SEARCH_FOOD)->AddRules(TTR_SEARCH_FOOD);
  AFind(PROC_SEARCH_SHADOW)->AddRules(TTR_SEARCH_SHADOW);
  AFind(PROC_REFLEX)->AddRules(TTR_REFLEX);
  AFind(PROC_ONE_STEP)->AddRules(TTR_ONE_STEP);
  AFind(PROC_WALK)->AddRules(TTR_WALK);
  AFind(PROC_SLEEP)->AddRules(TTR_SLEEP);
  AFind(PROC_EAT)->AddRules(TTR_EAT);
  AFind(PROC_ESCAPE)->AddRules(TTR_ESCAPE);
  AFind(PROC_MOVE_TO_OBSTACLE)->AddRules(TTR_MOVE_TO_OBST);

  printf("%s", Title);
  if(argc<3)
    error("Usage is: robotinifile needsinifile");
  char *robotinifile = argv[1];
  char *needsinifile = argv[2];

  //--------------------------------------------------------
  // Чтение конфигурации
  //--------------------------------------------------------
  ReadNeeds(needsinifile);

  //--------------------------------------------------------
  if(!Robot.Init(robotinifile))
    error("Init robot error");

  //--------------------------------------------------------
  // Инициализация узла
  ros::init(argc, argv, "tmusrv");
  ros::NodeHandle nsub1, nsub2, npub1, npub2;
  const char *sub_proc_topic = "proc_topic";
  const char *sub_ans_topic = "ardans_topic";
  const char *adv_senneed_topic = "senneed_topic";
  const char *adv_action_topic = "actions_topic";

  // Подписываемся на топик proc, второй параметр - объем кэша отправки
  ros::Subscriber pos_sub1 = nsub1.subscribe(sub_proc_topic, 1, GetProcMsg);
  ros::Subscriber pos_sub2 = nsub2.subscribe(sub_ans_topic, 1, GetAnsMsg);

  // Определение выходных топиков
  chatter_senneed_pub = npub1.advertise<msg_kvorum::senneed>(adv_senneed_topic, 1);
  chatter_action_pub = npub2.advertise<msg_rsaction::action>(adv_action_topic, 1);

  ROS_INFO("TMU Server started\n");

  sleep(1);
  printf("Test and setup... "); fflush(stdout);

  Robot.pub = chatter_action_pub;
  Robot.SetupParams();

  printf("Done.\n"); fflush(stdout);

  //--------------------------------------------------------
  // Основной цикл
  //--------------------------------------------------------

  w_main = erl::mvInitWindow(36, 92);
  wprintw(w_main, "%s", Title);
  nodelay(w_main, TRUE);
  wrefresh(w_main);

  ros::Duration(2).sleep();
  ros::Rate r(5); // 10
  long StartTime = emt::GetTimeSec();

  TAutomaton *fsm = AFind(PROC_ONE_STEP);
  FSMStack.Push(fsm);

  while(ros::ok())
  {
    CurrentTime = emt::GetTimeSec() - StartTime;
    ros::spinOnce();

    //------------------------------------------------------
    // Читаем все, что можно
    //------------------------------------------------------
    Robot.RequestAllSensors();

    EvaluateSensorsInfo();
    ShowRobotStatus(fsm);

    //------------------------------------------------------
    // Шаг работы автомата
    //------------------------------------------------------
    fsm = FSMStack.GetLast();
    if(!fsm) error("FSMStack.GetLast() error");

    int res = fsm->Step();
    if(res==-1) // Фатальная ошибка
      AError(fsm, "*** Step error");
    if(res==0) // Автомат завершил работу. Извлекаем предыдущий
    {
      fsm = FSMStack.Pop();
      if(FSMStack.Len()==0) // Стек пуст. Будем работать с пустым автоматом
      {
        TAutomaton *ca = AFind(PROC_ONE_STEP);
        FSMStack.Push(ca);
        ca->Reset();
      }
    }
    //------------------------------------------------------
    // Рефлексы
    //------------------------------------------------------
    if(VSenNeed.sen_reflex != REFLEX_NONE)
    {
      TAutomaton *ca = AFind(PROC_REFLEX);

      TAutomaton *predfsm = FSMStack.GetLast();
      if(ca!=predfsm)
      {
        FSMStack.Push(ca);
        ca->Reset();
      }
      fsm = ca;
    }
    r.sleep();
  }

  erl::mvEndWin();
  ROS_INFO("\nTMU Server terminated\n");
  return 0;
}
