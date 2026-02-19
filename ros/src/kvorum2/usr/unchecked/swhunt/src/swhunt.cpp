/**
 * \file
 * Пример многоагентной системы
 * rcX protocol
 * \author Robofob
 * \version 2.04
 *
 * \date 01.08.2013
 * \date LP 18.05.2016
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
#include <msg_kvorum/viz.h>

#include <msg_ans/ans.h>
#include <msg_rsaction/action.h>

#include "../../../../../arduino/libraries/i2cpro/i2cwctl.h"
#include "../../../../lib/erlib.h"
#include "../../../../lib/servlib.h"
#include "../../../lib/emt/emt.h"
#include "../../../lib/auto/auto.h"
#include "../../../lib/tmurobot/tmurobot.h"

using namespace std;

//------------------------------------------------------------------------------

const char *Title = "\nSWHunt 2.04\n";

WINDOW *w_main = NULL;

ros::Publisher chatter_action_pub;

long CurrentTime = 0;

struct TAgent: public TRobot
{
  msg_kvorum::senneed VSenNeed;
  int
    FreeWayDirection, // 0 - прямо, -1 - проход слева, +1 - проход справа
    FreeWayCPos,      // Центр тяжести
    FreeWayWeight;    // Вес
  TAgent(int id): TRobot(id)
  {
    FreeWayDirection = FreeWayCPos = 0;
  }
  void ShowStatus(int y);
  void EvaluateSensorsInfo(void);
};

//------------------------------------------------------------------------------

void TAgent::EvaluateSensorsInfo(void)
{
  static long pred_ctime = 0;
  TVAL rspot = TRobot::ReverseSpotSens?(TVAL)(255-(TVAL)TRobot::REC_SPOT()):(TVAL)TRobot::REC_SPOT();

  VSenNeed.sen_food = TRobot::rbSENSORS[SEN_FOOD].Put(rspot);
  VSenNeed.sen_danger = TRobot::rbSENSORS[SEN_DANGER].Put((TVAL)TRobot::REC_LIGHT());

  // Определяемся с углами дальномера
  FreeWayDirection = TRobot::FindLocObj(&FreeWayCPos, &FreeWayWeight);

  int ObstacleDirection, ObstacleCPos, ObstacleWeight;
  ObstacleDirection = TRobot::FindLocObj(&ObstacleCPos, &ObstacleWeight, 1);
  
  VSenNeed.sen_obstacle = TRobot::rbSENSORS[SEN_OBSTACLE].Put((TVAL)(FreeWayDirection==0?0:ObstacleWeight));

  // Рефлексы
  VSenNeed.sen_reflex = REFLEX_NONE;
  int rleft = (TRobot::REC_SHARP_FL()>TRobot::IR_LIM);
  int rright = (TRobot::REC_SHARP_FR()>TRobot::IR_LIM);
  if(rleft && rright)
    VSenNeed.sen_reflex = REFLEX_ALL;
  else
  if(rleft)
    VSenNeed.sen_reflex = REFLEX_LEFT;
  else
  if(rright)
    VSenNeed.sen_reflex = REFLEX_RIGHT;
}

void TAgent::ShowStatus(int y)
{
  #define ATTR 0

  erl::printfPos(w_main, y, 1,  ATTR, "A %d ===================================================", id);

  y++;
  TRobot::ShowReceptors(w_main,1, y);

  y+=3;
  // TSOP RC5
  erl::printfPos(w_main, y, 1,  ATTR, "TSOP (%d): ", TRobot::TSOPRC5.size());
  for(int i=0;i<TRobot::TSOPRC5.size();i++)
    wprintw(w_main, "%04X ", TRobot::TSOPRC5[i]);

  y+=1;
  // Локатор
  wmove(w_main, y, 1);
  int nloc = TRobot::Locator.size();
  for(int i=0;i<nloc;i++)
    wprintw(w_main, "%c", TRobot::Locator[i]?'*':'.');
  wprintw(w_main, "  %2d >> (dir: %2d pos: %2d w: %3d) Rf: %d", nloc, FreeWayDirection, FreeWayCPos, FreeWayWeight, VSenNeed.sen_reflex);
  wrefresh(w_main);
}

std::vector<TAgent*> Agents;

TAgent *AFind(int id)
{
  for(int i=0;i<Agents.size();i++)
    if(Agents[i]->id==id) return Agents[i];
  error("*** Agent #%d not found", id);
  return NULL;
}

//------------------------------------------------------------------------------
//
// Прием сообщения
//
//------------------------------------------------------------------------------
void GetAnsMsg(const msg_ans::ans &msg)
{
  TAgent *A = AFind(msg.id);
  A->AcceptMessage(msg);
}
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
  printf("%s", Title);
  if(argc<3)
    error("Usage is: robotinifile numagn");
  char *robotinifile = argv[1];

  int NumAgn = atoi(argv[2]);
  //--------------------------------------------------------
  // Инициализация узла
  ros::init(argc, argv, "swhunt");
  ros::NodeHandle nsub, npub1, npub2;

  // Подписываемся на топик ardans
  int inptopicnum = NumAgn*7;
  ros::Subscriber pos_sub = nsub.subscribe("ardans_topic", inptopicnum, GetAnsMsg);

  // Определение выходного топика
  int outtopicnum = NumAgn*7;
  chatter_action_pub = npub1.advertise<msg_rsaction::action>("actions_topic", outtopicnum);

  ROS_INFO("SWHunt started\n");

  //--------------------------------------------------------
  // Инициализация агентов
  //--------------------------------------------------------
  printf("Init agents\n"); fflush(stdout);
  for(int i=0;i<NumAgn;i++)
    Agents.push_back(new TAgent(i+1));

  // Чтение конфигурации
  for(int i=0;i<NumAgn;i++)
  {
    printf("Read %d/%d: ", i+1, NumAgn);
    if(!Agents[i]->Init(robotinifile))
      error("Init robot error");
    printf("ok\n");
    Agents[i]->pub = chatter_action_pub;
    Agents[i]->SetupParams();
  }

  printf("Done.\n"); fflush(stdout);

  //--------------------------------------------------------
  // Основной цикл
  //--------------------------------------------------------

  w_main = erl::mvInitWindow(38, 96);
  wprintw(w_main, "%s", Title);
  nodelay(w_main, TRUE);
  wrefresh(w_main);

  ros::Duration(2).sleep();
  ros::Rate r(10);
  long StartTime = emt::GetTimeMSec();
  long CTT = 0;
  while(ros::ok())
  {
    GLOB_PUB_CNT = 0;
    CurrentTime = emt::GetTimeMSec() - StartTime;
    ros::spinOnce();

    for(int i=0;i<NumAgn;i++)
    {
      // Читаем все, что можно
      Agents[i]->RequestAllSensors();

      Agents[i]->EvaluateSensorsInfo();
      if(i<4) // Будем показывать в лучшем случае первых 4 агентов
        Agents[i]->ShowStatus(3+i*7);

      // Шаг работы
      int act = PROC_NONE;
      // Рефлексы
      if(Agents[i]->VSenNeed.sen_reflex == REFLEX_LEFT)
      {
        act = PROC_GORIGHT;
      }
      else
      if(Agents[i]->VSenNeed.sen_reflex == REFLEX_RIGHT)
      {
        act = PROC_GOLEFT;
      }
      else
      if(Agents[i]->VSenNeed.sen_reflex == REFLEX_ALL)
      {
        act = PROC_GOLEFT;
      }
      else
      {
        // Анализ показаний локатора
        int nloc = Agents[i]->Locator.size();
        if(nloc)
        {
          if(Agents[i]->FreeWayWeight<1) // Никак не проехать
          {
            act = PROC_GOLEFT;
          }
          else
          {
            // Определяем свободное место
            switch(Agents[i]->FreeWayDirection)
            {
              case TMU_LOC_DIR_FWD:   // свободно
                act = PROC_GOFWD;
                break;
              case TMU_LOC_DIR_LEFT:  // слева
                act = PROC_GOLEFT;
                break;
              case TMU_LOC_DIR_RIGHT: // проход
                act = PROC_GORIGHT;
                break;
              case TMU_LOC_DIR_NONE:  // прохода нет
                act = PROC_GOLEFT;
                break;
            }
          }
        }
      }

      Agents[i]->Make(act, 1);
    }
    CTT++;
    char csc = (CTT%2==0)?'|':'-';
    erl::printfPos(w_main, 1, 1,  ATTR, "%c GLOB_PUB_CNT = %d  ", csc, GLOB_PUB_CNT);

    r.sleep();
  }

  erl::mvEndWin();
  ROS_INFO("\nSWHunt terminated\n");
  return 0;
}
