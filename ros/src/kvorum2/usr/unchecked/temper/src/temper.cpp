/*
  Эмоции и темперамент робота
  Version 2.02
     01.08.2013
  LP 06.04.2016
*/

#define _LINUX_

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

#include "../../../../lib/rcproto/tmucmd.h"
#include "../../../../lib/erlib.h"
#include "../../../../lib/servlib.h"
#include "../../../lib/emt/emt.h"

//----------------------------------------------------------
const char *Title = "\nEmoTemper CTL 2.01\n";

ros::Publisher chatter_proc_pub;
msg_kvorum::senneed VSenNeed;

WINDOW *w_main;
#define ATTR     0

// log-файл
FILE *LogFile = NULL;

long CurrentTime;

int AGENT_ID = 1; // Идентификатор агента, которым мы управляем

TVAL EMOTION = 0;

//----------------------------------------------------------
//
//----------------------------------------------------------
#define Y_SHOW_HELP      4
#define Y_SHOW_SENSORS   Y_SHOW_HELP+4
#define Y_SHOW_EMTER     Y_SHOW_SENSORS+6

//----------------------------------------------------------
//
//----------------------------------------------------------
int currProcedure = PROC_NONE;

void MakeAction(int procnum)
{
#define RTIME 5
  static int pred_proc = -1;
  static long predT = time(NULL);
  long cT = time(NULL);
  // Чтоб не повторять часто одно и то же действие
  if(procnum == pred_proc)
  {
    if(cT-predT<RTIME)
      return;
    predT = cT;
  }

  pred_proc = procnum;

  msg_kvorum::proc msg;
  msg.id = AGENT_ID;
  msg.proc = procnum;
  chatter_proc_pub.publish(msg);

  erl::printfPos(w_main, 2, 1, ATTR, "-- [%ld] procnum = %d %-12s ", cT, procnum, ProcName2Str(procnum));
}

void WriteLogFileHeader(void)
{
  if(!LogFile) return;
  fprintf(LogFile, "Time;"
                   "sen_food;sen_danger;sen_hungry;sen_obstacle;"
                   "need_food;"
                   "currProcedure;EMOTION;"
                   "sen_excit;sen_inhibit;"
                   "need_comfort;need_save;"
                   "gate_food;gate_obstacle;gate_danger;"
                   "PROCNAMES;\n");
}

void WriteLogFile(void)
// Запись в log-файл
{
  static long predcds = 0;
  if(!LogFile) return;
  long cds = emt::GetTimeMSec();
  if(cds-predcds>=param::LogFileTMs)
    predcds = cds;
  else
    return;
  fprintf(LogFile, "%ld;", cds);
  fprintf(LogFile, "%d;%d;%d;%d;", VSenNeed.sen_food, VSenNeed.sen_danger, VSenNeed.sen_hungry, VSenNeed.sen_obstacle);
  fprintf(LogFile, "%d;",          VSenNeed.need_food);
  fprintf(LogFile, "%d;%d;",       currProcedure, EMOTION);
  fprintf(LogFile, "%d;%d;",       VSenNeed.sen_excit, VSenNeed.sen_inhibit);
  fprintf(LogFile, "%d;%d;",       VSenNeed.need_comfort, VSenNeed.need_save);
  fprintf(LogFile, "%d;%d;%d;",    gate_food, gate_obstacle, gate_danger);
  fprintf(LogFile, "%s",           ProcName2Str(currProcedure));
  fprintf(LogFile, "\n");
}

void GetSenNeedMsg(const msg_kvorum::senneed &msg)
{
   VSenNeed = msg;
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void ReadExperimentParams(char *inifile)
{
  l_string s;
  FILE *f=fopen(inifile,"r");
  if(!f)
    error("open ini file %s error", inifile);

  // * coeff_em,  coeff_fb
  if(!SkipRemarkLine(f, s))
    error("format error 5");
  sscanf(s,"%f %f", &param::coeff_em,  &param::coeff_fb);

  // * Период записи в log-файл, мс
  param::LogFileTMs = ReadInt(f);

  param::LogFileName[0] = 0;
  // * Префикс log-файла
  if(SkipRemarkLine(f, s))
  {
    time_t rawtime;
    struct tm * tdt;

    time (&rawtime);
    tdt = localtime (&rawtime);

    l_string sd;
    //yyyy-mm-dd-hh-nn-ss
    sprintf(sd,"%04d-%02d-%02d-%02d-%02d-%02d",
      tdt->tm_year+1900, tdt->tm_mon+1, tdt->tm_mday+1, tdt->tm_hour, tdt->tm_min, tdt->tm_sec);
    strcat(s,sd);
    strcat(s,".csv");
    strcpy(param::LogFileName, s);
  }
  fclose(f);
}

int main(int argc, char *argv[])
{
  printf("%s", Title);
  if(argc<2)
    error("Usage is: sysinifile");
  char *inifile = argv[1];

  //--------------------------------------------------------
  // Чтение конфигурации
  //--------------------------------------------------------
  ReadExperimentParams(inifile);
  if(param::LogFileName[0])
  {
    if((LogFile = fopen(param::LogFileName, "a"))==NULL)
      error("Open file %s error", param::LogFileName);
  }
  WriteLogFileHeader();

  printf("Start main proc...");

  // Инициализация кольцевых буферов
  InitRB();

  //--------------------------------------------------------
  // Инициализация узла
  ros::init(argc, argv, "temper");
  ros::NodeHandle nsub, npub;

  const char *adv_proc_topic = "proc_topic";
  const char *sub_senneed_topic = "senneed_topic";

  // Подписываемся на топик senneed, второй параметр - объем кэша отправки
  ros::Subscriber pos_sub = nsub.subscribe(sub_senneed_topic, 1, GetSenNeedMsg);

  // Определение выходных топиков
  chatter_proc_pub = npub.advertise<msg_kvorum::proc>(adv_proc_topic, 1);

  ROS_INFO("TEMPER started\n");

  w_main = erl::mvInitWindow(23, 90);
  wprintw(w_main, "%s", Title);
  nodelay(w_main, TRUE);
  wrefresh(w_main);

  erl::printfPos(w_main, Y_SHOW_HELP,   1, ATTR, "-- b:Beep, Sp:Stop, w/s/a/d:Move CR:auto/manual");
  erl::printfPos(w_main, Y_SHOW_HELP+1, 1, ATTR, "-- Proc: 1-Search Food 2-Eat 3-Escape 4-Search Shadow 5-Sleep 6-MvObst 7-Walk 0-EMPTY");

  //--------------------------------------------------------
  // Основной цикл
  //--------------------------------------------------------
  ros::Duration(2).sleep();
  ros::Rate r(10);
  int rcnt = 0;
  long StartTime = emt::GetTimeMSec();
  int eoj = 0;
  int ManualRegime = 1;
  while(ros::ok() && !eoj)
  {
    CurrentTime = emt::GetTimeMSec() - StartTime;
    ros::spinOnce();
    int wascmd = 1;
    char c = wgetch(w_main);
    switch(c)
    {
      case 27: eoj = 1; break;
      case 'b': currProcedure = PROC_BEEP; break;
      case ' ': currProcedure = PROC_STOP; break;
      case 'w': currProcedure = PROC_GOFWD; break;
      case 's': currProcedure = PROC_GOBACK; break;
      case 'a': currProcedure = PROC_GOLEFT; break;
      case 'd': currProcedure = PROC_GORIGHT; break;
      case 'W': currProcedure = PROC_STEPFWD; break;
      case 'S': currProcedure = PROC_STEPBACK; break;
      case 'A': currProcedure = PROC_STEPLEFT; break;
      case 'D': currProcedure = PROC_STEPRIGHT; break;
      //case '+': Robot.SetSpeed(Robot.GetSpeed()+5); break;
      //case '-': Robot.SetSpeed(Robot.GetSpeed()-5); break;
      case '1': currProcedure = PROC_SEARCH_FOOD; break;
      case '2': currProcedure = PROC_EAT; break;
      case '3': currProcedure = PROC_ESCAPE; break;
      case '4': currProcedure = PROC_SEARCH_SHADOW; break;
      case '5': currProcedure = PROC_SLEEP; break;
      case '6': currProcedure = PROC_MOVE_TO_OBSTACLE; break;
      case '7': currProcedure = PROC_WALK; break;
      case '0': currProcedure = PROC_STOP; break;
      case 10:
      case 13:
        ManualRegime = !ManualRegime;
        wascmd = 0;
        break;
      default:
        wascmd = 0;
    }
    if(wascmd)
    {
      MakeAction(currProcedure);
      ManualRegime = 1;
    }

    if(ManualRegime)
      erl::printfPos(w_main, Y_SHOW_HELP+2, 1, ATTR, "** MANUAL");
    else
      erl::printfPos(w_main, Y_SHOW_HELP+2, 1, ATTR, "** AUTO  ");

    emt::ShowSensorsNeeds(w_main, Y_SHOW_SENSORS, VSenNeed);
    if(ManualRegime) continue;
    //------------------------------------------------------
    // Эмоции
    //------------------------------------------------------
    // Вычисление выходных сигналов вентилей (Gates values)
    //------------------------------------------------------
    // gate_food :- sen_food + E(EAT)
    gate_food = rbGATE_FOOD.Put_Norm((TVAL)(VSenNeed.sen_food + etEM[PROC_EAT]));

    // gate_obstacle :- sen_obstacle + E(MOVE_TO_OBSTACLE) + E(WALK)
    gate_obstacle = rbGATE_OBSTACLE.Put_Norm((TVAL)(VSenNeed.sen_obstacle + etEM[PROC_MOVE_TO_OBSTACLE] + etEM[PROC_WALK]));

    // gate_danger :- sen_danger + E(ESCAPE)
    gate_danger = rbGATE_DANGER.Put_Norm((TVAL)(VSenNeed.sen_danger + etEM[PROC_ESCAPE]));

    //------------------------------------------------------
    // Вычисление активностей процедур (Procedures values)
    //------------------------------------------------------
    memset(etPROC,0,sizeof(etPROC));
    float excitation = VSenNeed.sen_excit/(float)MAX_SIGN;

    // Поедание пищи: proc_eat_food :- need_food & gate_food & sen_hungry
    etPROC[PROC_EAT] = rbPROC[PROC_EAT].Put((TVAL)(excitation*(TVAL)min3(VSenNeed.need_food, gate_food, VSenNeed.sen_hungry)));

    // Движение к препятствию: proc_move_to_obstacle :- gate_obstacle & need_food & !gate_food
    etPROC[PROC_MOVE_TO_OBSTACLE] = rbPROC[PROC_MOVE_TO_OBSTACLE].Put((TVAL)(excitation*min3(gate_obstacle, VSenNeed.need_food,
                                                                           (TVAL)(MAX_SIGN-gate_food))));

    // Свободное блуждание: proc_walk :- !gate_obstacle & need_comfort
    etPROC[PROC_WALK] = rbPROC[PROC_WALK].Put((TVAL)(excitation*min2((TVAL)(MAX_SIGN-gate_obstacle), VSenNeed.need_comfort)));

    // Убегание от препятствия: proc_run_away :- need_save & gate_obstacle
    etPROC[PROC_ESCAPE] = rbPROC[PROC_ESCAPE].Put((TVAL)(excitation*min2(VSenNeed.need_save, gate_obstacle)));

    // Поиск тени: PROC_SEARCH_SHADOW :- need_comfort & gate_danger
    etPROC[PROC_SEARCH_SHADOW] = rbPROC[PROC_SEARCH_SHADOW].Put((TVAL)(excitation*min2(VSenNeed.need_comfort,gate_danger)));

    // Сон: PROC_SLEEP :- need_comfort & !gate_danger
    etPROC[PROC_SLEEP] = rbPROC[PROC_SLEEP].Put((TVAL)(excitation*min2(VSenNeed.need_comfort,(TVAL)(MAX_SIGN-gate_danger))));

    // Поиск пищи: PROC_SEARCH_FOOD :- need_food & !gate_food & sen_hungry
    etPROC[PROC_SEARCH_FOOD] = rbPROC[PROC_SEARCH_FOOD].Put((TVAL)(excitation*min3(VSenNeed.need_food,(TVAL)(MAX_SIGN-gate_food), VSenNeed.sen_hungry)));

    //------------------------------------------------------
    // Выходные значения
    //------------------------------------------------------
    TVAL V[PROC_NUM];
    for(int i=0;i<PROC_NUM;i++)
    {
      int S = 0;
      for(int j=0;j<PROC_NUM;j++)
        if(i!=j) S+=etOUT[j];
      V[i] = rbOUT[i].Put_Norm((TVAL)(etPROC[i] - param::coeff_fb*S));
    }
    for(int i=0;i<PROC_NUM;i++)
      etOUT[i] = V[i];

    //------------------------------------------------------
    // FACT Out
    // Выводить вектор etFACT не надо - он неинтересный
    //------------------------------------------------------
    int nmax = 0;
    for(int i=0;i<PROC_NUM;i++)
    {
      if(etOUT[i]>etOUT[nmax]) nmax = i;
      etFACT[i] = 0;
    }
    etFACT[nmax] = MAX_SIGN;
    currProcedure = nmax;
    //------------------------------------------------------
    // Вычисляем эмоции
    //------------------------------------------------------
    EMOTION = 0;
    for(int i=0;i<PROC_NUM;i++)
    {
      etEM[i] = (TVAL)(param::coeff_em*(etFACT[i] - etPROC[i]));
      EMOTION+=etEM[i];
    }

    //------------------------------------------------------

    int y = Y_SHOW_EMTER;
    erl::printfPos(w_main, y++, 1, ATTR,  "NPROC     EMOTION   G.Food  G.Obst  G.Dang");
    erl::printfPos(w_main, y++, 1, ATTR,  "%-6s    %3d       %3d     %3d     %3d  ",
            ProcName2Str(currProcedure), EMOTION, gate_food, gate_obstacle, gate_danger);
    y++;
    erl::printfPos(w_main, y++, 1, ATTR, "PROC: ");
    for(int i=0;i<PROC_NUM;i++)
      wprintw(w_main, "%6s ", ProcName2Str(i));

    erl::printfPos(w_main, y++, 1, ATTR, "      ");
    for(int i=0;i<PROC_NUM;i++)
      wprintw(w_main, "%6d ", etPROC[i]);

    erl::printfPos(w_main, y++, 1, ATTR, "EM:   ");
    for(int i=0;i<PROC_NUM;i++)
      wprintw(w_main, "%6d ", etEM[i]);

    erl::printfPos(w_main, y++, 1, ATTR, "OUT:  ");
    for(int i=0;i<PROC_NUM;i++)
      wprintw(w_main, "%6d ", etOUT[i]);

    //------------------------------------------------------
    // Действия
    // Выполнение поведенческой подпрограммы currProcedure
    //------------------------------------------------------
    MakeAction(currProcedure);
    WriteLogFile();

    r.sleep();
  }
  MakeAction(PROC_STOP);
  if(LogFile)
    fclose(LogFile);
  erl::mvEndWin();
  ROS_INFO("\nTEMPER terminated\n");
}
