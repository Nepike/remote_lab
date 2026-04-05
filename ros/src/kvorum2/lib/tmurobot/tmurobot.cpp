/*
 * TMU robot library
 * Temperament tools and elements
 * 08.07.2013
 * LP 11.06.2016
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <tinyxml.h>

#include "tmurobot.h"
using namespace std;

//------------------------------------------------------------------------------
/// Преобразуем в вектор данных, убирая лишние заголовочные байты
void msg2data(const std::vector<int> &m, std::vector<int> &data)
{
  unsigned char datalen = m[POS_LEN];
  int pkglen = datalen+POS_LEN+1;

  if(pkglen>=RC_MAX_BUFF)
    error("msg2data: packet len error: %d", datalen);

  for(int i=0;i<datalen;i++)
    data.push_back(m[i+POS_DATA]);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

void XMLGet(TiXmlElement *base, const char *name, int &val)
{
  TiXmlElement *obj = base->FirstChildElement(name);
  if(!obj)
    error("can't find '%s' element", name);
  string text = obj->GetText();
  if(text=="")
    error("field '%s' is empty", name);
  int n;
  if(sscanf(text.c_str(),"%d", &n)!=1)
    error("field '%s' format error", name);
  val = n;
}

void XMLGet(TiXmlElement *base, const char *name, BYTE &val)
{
  int n;
  XMLGet(base, name, n);
  val = n;
}

int TRobot::Init(const char *filename)
{
  TiXmlDocument doc(filename);

  bool loadOkay = doc.LoadFile();

  if (!loadOkay)
    error("Could not load  %s file. Error='%s'.\n", filename, doc.ErrorDesc());

  TiXmlElement *base = doc.FirstChildElement("Robot");
  if (!base)
    error("File %s format error",filename);

  // * Скорость робота
  XMLGet(base, "Speed", Speed);

  // * Флаг блокирования двигательных функций
  XMLGet(base, "LockMove", LockMovFunc);

  // * Режим инверсии датчика пятна (еда): 0 - нет инверсии, 1 - инверсия
  XMLGet(base, "InverseSpot", ReverseSpotSens);

  // * Шаг угла поворота
  XMLGet(base, "ANG_STEP", ANG_STEP);

  // * Порог датчика Sharp для срабатывания рефлекса IR_LIM
  XMLGet(base, "IR_LIM", IR_LIM);

  // * Порог датчика Sharp локатора
  XMLGet(base, "IR_C_LIM", IR_C_LIM);

  // * Порог датчика пятна (еда)
  XMLGet(base, "SPOT_LIM", SPOT_LIM);

  return 1;
}

int GLOB_PUB_CNT = 0;
void TRobot::Publish(int act, int arg1, int arg2, int arg3)
{
  /// Сообщение выходного топика
  msg_rsaction::action msg;
  msg.team_id = 1;
  msg.agent_id =  id;
  msg.action = act; // Command
  msg.arg1 = arg1;  // Argument 1
  msg.arg2 = arg2;  // Argument 2
  msg.arg3 = arg3;  // Argument 3
  msg.data.clear();
  pub.publish(msg);
  GLOB_PUB_CNT++;
}

//------------------------------------------------------------------------------
// Двигательные функции
//------------------------------------------------------------------------------

void TRobot::Beep(int n)
{
  for(int i=0;i<n;i++)
  {
    Publish(CMD_BEEP_ON);
    delay_ms(TACTD);
    Publish(CMD_BEEP_OFF);
    delay_ms(TACTD);
  }
}

void TRobot::StepFwd(int n)
{
  if(LockMovFunc) return;
  Publish(CMD_FWD2);
}

void TRobot::StepBack(int n)
{
  if(LockMovFunc) return;
  Publish(CMD_BACK2);
}

void TRobot::StepLeft(int n)
{
  if(LockMovFunc) return;
  Publish(CMD_LEFT2);
}

void TRobot::StepRight(int n)
{
  if(LockMovFunc) return;
  Publish(CMD_RIGHT2);
}

void TRobot::ShowReceptors(WINDOW *w, int x, int y)
{
  #define ATTR 0
  erl::printfPos(w, y++, x, ATTR, "Rec: ShFL  ShFR  ShSL  ShSR  ShC   Light Excit Inhib Spot");
  erl::printfPos(w, y++, x, ATTR, "     %3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d",
         REC_SHARP_FL(), REC_SHARP_FR(), REC_SHARP_SL(), REC_SHARP_SR(), REC_SHARP_C(),
         REC_LIGHT(), REC_EXCIT(), REC_INHIBIT(), REC_SPOT());

  erl::printfPos(w, y++, x, ATTR, "I2CData: ");
  // Берем только АЦП
  int n = Dataserver.size();
  if(n>8) n = 8;
  for(int i=0;i<n;i++)
    wprintw(w, "%3d ", Dataserver[i]);
}

//
// Возвращаемые значения: TMU_LOC_DIR_FWD - искомое прямо, TMU_LOC_DIR_LEFT - слева, TMU_LOC_DIR_RIGHT - справа, TMU_LOC_DIR_NONE - не найдено
//   search_obs:
//     0 - определяем свободное пространство
//     1: определяем занятое пространство
// cpos - позиция центра тяжести найденного объекта
// weight - вес найденного объекта
int TRobot::FindLocObj(int *cpos, int *weight, int search_obs)
{
  int sw, swi;
  short n, cdir;
  *weight = 0;
  sw = swi = 0;
  int nloc = Locator.size();

  int q = 0;
  // Ищем максимально длинный участок
  int n0 = 0;
  int len = 0;
  int maxlen = 0;
  int nmax = 0;
  for(n=0;n<nloc;n++)
  {
    int L = search_obs? (!Locator[n]):(Locator[n]);
    if(q==0)
    {
      if(L==0)
      {
        n0 = n;
        len = 1;
        q = 1;
      }
    }
    else
    {
      if(L==0) len++;
      else
      {
        if(len>maxlen)
        {
          maxlen = len;
          nmax = n0;
        }
        q = 0;
      }
    }
  }
  if(len>maxlen)
  {
    maxlen = len;
    nmax = n0;
  }
  *weight = maxlen;
  *cpos = nmax+maxlen/2;
  int delta = (nloc/10);

  cdir = TMU_LOC_DIR_FWD;
  if(*cpos<nloc/2-delta)
    cdir = TMU_LOC_DIR_LEFT;
  if(*cpos>nloc/2+delta)
    cdir = TMU_LOC_DIR_RIGHT;
  if(*weight==0)
    cdir = TMU_LOC_DIR_NONE;

  return cdir;
}

#define NCNT 4
void TRobot::RequestAllSensors(int immediate)
{
  if(immediate)
  {
    RequestSensors();
    RequestRegisters();
    RequestUsrData();
    RequestI2CData();
    RequestRC5Data();
  }
  else
  {
    switch(cnt_act)
    {
      case 0: RequestSensors(); break;
      case 1: RequestRegisters(); break;
      case 2: RequestUsrData(); break;
      case 3: RequestI2CData(); break;
      case 4: RequestRC5Data(); break;
    }
    cnt_act++;
    if(cnt_act>NCNT) cnt_act = 0;
  }
}

// Выполнение примитивной поведенческой подпрограммы cmd - разового действия
void TRobot::Make(int cmd, int wait_cnt)
{
  long ct = time(NULL);
  static long pred_time = 0;
  #define dT 2

  if(cmd==LAST_ROB_CMD && (ct-pred_time)<dT) return;
  pred_time = ct;

  switch(cmd)
  {
    case PROC_STOP:   Stop(); break;
    case PROC_GOFWD:  GoFwd(); break;
    case PROC_GOBACK: GoBack(); break;
    case PROC_GOLEFT: GoFastLeft(); break;
    case PROC_GORIGHT:GoFastRight(); break;
    case PROC_STEPFWD: StepFwd(1); break;
    case PROC_STEPBACK: StepBack(1); break;
    case PROC_STEPLEFT: StepLeft(1); break;
    case PROC_STEPRIGHT: StepRight(1); break;
    case PROC_EAT:              // Поедание пищи
      Publish(CMD_DEBUG); break;
    case PROC_BEEP:
    case PROC_ESCAPE:           // Убегание от препятствия
    case PROC_MOVE_TO_OBSTACLE: // Движение к препятствию
    case PROC_SEARCH_SHADOW:    // Поиск тени
    case PROC_SLEEP:
    case PROC_WALK:
    case PROC_SEARCH_FOOD:
    case PROC_NONE:
      break;
    default:
      error("MakeAction: unknown cmd: %d", cmd);
   }
  LAST_ROB_CMD = cmd;
}

// Обработка принятого сообщения
void TRobot::AcceptMessage(const msg_ans::ans &msg)
{
  std::vector<int> rdata;
  msg2data(msg.data, rdata);

  if(msg.result==CMD_ANS_GET_SENS)
  {
    Receptors = rdata;
  }
  if(msg.result==CMD_ANS_GET_USR_DATA) // Локатор
  {
    // Переворачиваем (так надо: углы по-другому отсчитываются)
    Locator = rdata;
    int n = rdata.size();
    for(int i=0;i<n;i++)
      Locator[n-i-1] = (rdata[i]>IR_C_LIM);
  }
  if(msg.result==CMD_ANS_GET_ALL_REG)
  {
    Registers = rdata;
  }
  if(msg.result==CMD_ANS_GET_I2C_DATA) // Сначала идет адрес i2c-устройства
  {
    int i2caddr = rdata[0];
    rdata.erase(rdata.begin() + 0);    // Удаляем первый элемент
    if(i2caddr == I2CDataServer::ADDR)
      Dataserver = rdata;
    if(i2caddr == RC5Server::ADDR)
      TSOPRC5 = rdata;
  }
}
