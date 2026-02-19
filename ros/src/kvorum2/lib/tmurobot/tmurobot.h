/*
 * TMU robot library
 * Temperament tools and elements
 * 08.07.2013
 * LP 11.06.2016
*/

#ifndef _TMUROBOT_H_
#define _TMUROBOT_H_

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

#include <curses.h>
#include <tinyxml.h>

#include "../../../lib/erlib.h"
#include "../../../lib/servlib.h"

#include "../../../lib/rcproto/rcproto2.h"
#include "../../../lib/rcproto/tmucmd.h"
#include "../../arduino/libraries/i2cpro/i2cwctl.h"

#include "../../lib/emt/emt.h"

#ifndef BYTE
  typedef unsigned char BYTE;
#endif


// Сенсоры
#define SEN_NUM 16

enum { SEN_FOOD, SEN_DANGER, SEN_OBSTACLE };
enum { REFLEX_NONE, REFLEX_LEFT, REFLEX_RIGHT, REFLEX_ALL };

// Направления для FindLocObj
#define TMU_LOC_DIR_FWD    0
#define TMU_LOC_DIR_LEFT  -1
#define TMU_LOC_DIR_RIGHT  1
#define TMU_LOC_DIR_NONE   2

struct TRobot
{
  int LockMovFunc; // Флаг блокирования двигательных функций
  int TACTD;       // Время для шага действия, мс
  int Speed;
  // Режим инверсии датчика пятна (еда): 0 - нет инверсии, 1 - инверсия
  int ReverseSpotSens;  
  BYTE ANG_STEP; // Шаг угла поворота
  BYTE version;
  BYTE id;
  // Пороги
  BYTE IR_LIM;    // Порог датчика Sharp для срабатывания рефлекса
  BYTE IR_C_LIM;  // Порог датчика Sharp локатора
  BYTE SPOT_LIM;  // Порог датчика пятна (еда)

  TRingBuffer rbSENSORS[SEN_NUM];
  int LAST_ROB_CMD;

private:
  int cnt_act;

public:

  std::vector<int> Locator;    // Локатор
  std::vector<int> TSOPRC5;    // TSOP
  std::vector<int> Registers;  // Регистры
  std::vector<int> Receptors;  // Рецепторы
  std::vector<int> Dataserver; // Данные от i2c-сервера

  ros::Publisher pub;

  TRobot(int nid)
  {
    id = nid;
    LockMovFunc = 0;
    TACTD = 500;
    Speed = 1;
    ReverseSpotSens = 0;
    ANG_STEP  = 2;
    version = 0;
    IR_LIM = 50;
    IR_C_LIM = 60;
    SPOT_LIM = 100;
    for(int i=0;i<SEN_NUM;i++)
      rbSENSORS[i].Init(RBLEN);
    LAST_ROB_CMD = PROC_NONE;
    cnt_act = 0;
  }
  ~TRobot()
  {
    Publish(CMD_SET_REG, REG_LOC_ENABLE, 0); // Выключить локатор
    Stop();
  }

  int Init(const char *filename);

  void Publish(int act, int arg1 = 0, int arg2 = 0, int arg3 = 0);
  // Обработка принятого сообщения
  void AcceptMessage(const msg_ans::ans &msg);

  void RequestSensors(void) { Publish(CMD_GET_SENS); };
  void RequestRegisters(void) { Publish(CMD_GET_ALL_REG); };
  void RequestUsrData(void) {  Publish(CMD_GET_USR_DATA); };
  void RequestI2CData(void) { Publish(CMD_GET_I2C_DATA, I2CDataServer::ADDR, I2CDataServer::DATALEN); };
  void RequestRC5Data(void) { Publish(CMD_GET_I2C_DATA, RC5Server::ADDR, RC5Server::DATALEN); };

  // Запрос состояния сенсоров
  void RequestAllSensors(int immediate = 0);
  // Выполнение примитивной поведенческой подпрограммы cmd - разового действия
  void Make(int cmd, int wait_cnt = 0);

  //
  // Возвращаемые значения: TMU_LOC_DIR_FWD - искомое прямо, TMU_LOC_DIR_LEFT - слева, TMU_LOC_DIR_RIGHT - справа, TMU_LOC_DIR_NONE - не найдено
  //   search_obs:
  //     0 - определяем свободное пространство
  //     1: определяем занятое пространство
  // cpos - позиция центра тяжести найденного объекта
  // weight - вес найденного объекта
  int FindLocObj(int *cpos, int *weight, int search_obs = 0);

  void ShowReceptors(WINDOW *w, int x, int y);
  int GetSpeed(void) { return Speed; };
  void SetSpeed(int sp)
  {
    Speed = sp;
    Publish(CMD_SET_REG, REG_SPEED, (BYTE)Speed);
  }

  void SetupParams(void)
  {
    Stop();
    Publish(CMD_SET_REG, REG_ANG_STEP, ANG_STEP);
    Publish(CMD_SET_REG, REG_SPEED, (BYTE)Speed);
    Publish(CMD_SET_REG, REG_LOC_ENABLE, 1);       // Включить локатор
    //Beep(1);
  }
  int getUSRDATA(int n) { return (n<Receptors.size()) ? Receptors[n] : 0; }
  int getI2CDATA(int n) { return (n<Dataserver.size()) ? Dataserver[n] : 0; }
  //------------------------------------------------------------------------------
  // Первичные рецепторы
  //------------------------------------------------------------------------------

  int REC_SHARP_FL(void) { return getUSRDATA(6); }
  int REC_SHARP_FR(void) { return getUSRDATA(7); }
  int REC_SHARP_C(void) { return getUSRDATA(3); }

  int REC_SHARP_SL(void) { return getUSRDATA(1); }
  int REC_SHARP_SR(void) { return getUSRDATA(2); }

  int REC_LIGHT(void) { return getI2CDATA(3); }
  int REC_EXCIT(void) { return getI2CDATA(0); }
  int REC_INHIBIT(void) { return getI2CDATA(1); }
  int REC_DEBUG(void) { return getI2CDATA(8); }
  // Датчик пятна (еда)
  int REC_SPOT(void) { return ((getI2CDATA(2)>SPOT_LIM)?255:0); }

  // Двигательные функции
  void Beep(int n);
  void Stop(void) { Publish(CMD_STOP); };
  void GoFwd(void) { if(!LockMovFunc) Publish(CMD_FWD); }
  void GoBack(void) { if(!LockMovFunc) Publish(CMD_BACK); }
  void GoLeft(void) { if(!LockMovFunc) Publish(CMD_LEFT); }
  void GoRight(void) { if(!LockMovFunc) Publish(CMD_RIGHT); }
  void GoFastLeft(void) { if(!LockMovFunc) Publish(CMD_FAST_LEFT); }
  void GoFastRight(void) { if(!LockMovFunc) Publish(CMD_FAST_RIGHT); }

  void StepFwd(int n);
  void StepBack(int n);
  void StepLeft(int n);
  void StepRight(int n);
};


/// Преобразуем в вектор данных, убирая лишние заголовочные байты
void msg2data(const std::vector<int> &m, std::vector<int> &data);

extern int GLOB_PUB_CNT;

#endif
