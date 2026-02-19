/**
 * ER Library
 * Сервисная библиотека
 * \author Robofob
 * \version 1.11
 * \date 07.07.2014
 * \date LP 07.09.2016
*/

#ifndef _ERLIB_H_
#define _ERLIB_H_

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sstream>
#include <curses.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <msg_ans/ans.h>
#include <msg_senfield/senfield.h>
#include "exec_wrapper/exec_wrapper.h"

//----------------------------------------------------------
//
//----------------------------------------------------------
namespace erl
{

void mwSetAttr(WINDOW *w, char attr);
WINDOW *mvInitWindow(int Height = 25, int Width = 120);
void mvEndWin(void);
void printfAttr(WINDOW *w, char attr, const char *fmt, ...);
void printfPos(WINDOW *w, int y, int x, char attr, const char *fmt, ...);

//----------------------------------------------------------
//
//----------------------------------------------------------
//enum TSHOWANSREG { TSHANS_DATA = 0x01, TSHANS_CMD = 0x02 };
enum  { TSHANS_DATA = 0x01, TSHANS_CMD = 0x02 };

void AnsShow(const char *title, const msg_ans::ans &msg, int showreg);
void AnsShowPos(const char *title, const msg_ans::ans &msg, WINDOW *w, int y, int x, char attr, int showreg);

/// Вывод датчика щели (Kinect)
void ShowSlot(const char *title, const msg_ans::ans &msg, WINDOW *w, int y, int x, char attr);

//----------------------------------------------------------
//
//----------------------------------------------------------
int ExecERCommand(const msg_ans::ans &msg, ros::Publisher &ard_pub, ros::Publisher &pll_pub);
int ExecERCommand(const msg_ans::ans &msg, int addr, ros::Publisher &ard_pub, ros::Publisher &pll_pub, int cmd, int arg1 = 0, int arg2 = 0);

//----------------------------------------------------------
//
//----------------------------------------------------------
/**
 * ErCommand
 * Команды ans.result ans.size
 */
enum ErCommand
{
  ERCMD_STOP = 1,
  ERCMD_GOFWD,
  ERCMD_GOBACK,
  ERCMD_GOLEFT,
  ERCMD_GOFASTLEFT,
  ERCMD_GORIGHT,
  ERCMD_GOFASTRIGHT,
  ERCMD_BEEP,
  ERCMD_BEEP_ON,
  ERCMD_BEEP_OFF,

  ERCMD_GOFWD2,
  ERCMD_GOBACK2,         //12
  ERCMD_GOLEFT2,
  ERCMD_GOFASTLEFT2,
  ERCMD_GORIGHT2,
  ERCMD_GOFASTRIGHT2,

  ERCMD_ARD_GET_SENS,    // 17
  ERCMD_ARD_GET_ALL_REG, // 18
  ERCMD_SET_REG,

  ERCMD_PLL_GET_SENS,    // 20

  ERCMD_STOP_SCRIPT,     // 21
  ERCMD_START_SCRIPT,    // 22
  ERCMD_I2C,             // 23
  ERCMD_GET_I2C,         // 24
  ERCMD_SET_SPEED,       // NOTE: KOSTYL

  ERCMD_SET_EEP_REG
};

//----------------------------------------------------------
//
//----------------------------------------------------------
/**
  Описание сенсоров
  <Имя> <Префикс> <суффикс> <Y> <X> <src> <offs> <attr_pref> <attr_suff> <attr_val>
     <Y>, <X> - координаты
     <src> - источник данных: 0 - Arduino, 1 - Pololu
     <offs> - смещение в массиве data
    <attr_pref> <attr_suff> <attr_val>  - атрибуты (HEX)
*/

enum {
  SRC_ARD_DATA = 0, /// 0 - Arduino data
  SRC_ARD_REGS,     /// 1 - Arduino regs
  SRC_ARD_STAT,     /// 2 - Arduino status
  SRC_PLL_DATA,     /// 3 - Pololu
  SRC_FACEDETECTOR, /// 4 - Face Detector
  SRC_QRCODE        /// 5 - QRCode
};

struct TSenDescr
{
  std::string name; /// <Имя>
  std::string prefix, vformat, suffix; /// <Префикс> <Формат> <Cуффикс>
  int y, x;         /// Координаты
  int src;          /// Источник данных (SRC_ARD_DATA, SRC_ARD_REGS, SRC_ARD_STAT, PLL_DATA)
  int offs;         /// Смещение в массиве data
  char attr_pref, attr_suff, attr_val; // Атрибуты (HEX)
};

/// Описание вывода сенсоров
extern std::vector<TSenDescr> SensorsDescrList;

void ReadSensorsDescrList(char *fname);
void ShowSensors(WINDOW *w, const msg_senfield::senfield &msg);

/// Получить значение датчика по его имени
/// 1 - все хорошо, 0 - элемент не определен
int GetSenVal(std::string name, const msg_senfield::senfield &msg, int &dat);

/// Получить значение датчика по его порядковому номеру
/// 1 - все хорошо, 0 - элемент не определен
int GetSenVal(int sid, const msg_senfield::senfield &msg, int &dat);

//----------------------------------------------------------
//
//----------------------------------------------------------
/** Проверка контроллеров
 *  0 - всех хорошо
 *  1 - не работает Arduino
 *  2 - не работает Pololu
 *  3 - не работают оба контроллера
 */
int CheckServers(WINDOW *w, int y, int x, const msg_senfield::senfield &msg);

void Say(const char *msg);

void ReadText(const char *filename);

void RunCommand(const char *command);

void Alarm(WINDOW *w, const char *msg, int iterations = 1);

int kbhit(void);

}

/**
 * Кольцевой буфер
 */
#define MAX_RBUF_LEN 4 // 8 10
struct TRBuffer
{
  int pos;
  int buf[MAX_RBUF_LEN];
  int size;
  TRBuffer(int sz=MAX_RBUF_LEN) 
  {
    pos = 0;
    memset(buf, 0, sizeof(buf));
    SetSize(sz);
  }
  void Push(int val) // Записать данные в буфер
  {
    buf[pos] = val;
    pos++;
    if(pos>=size) pos = 0;
  }
  int GetAvr(void); // Получить среднее значение
  int SetSize(int sz)
  {
    size = sz;
    if(size<=0) size=1;
    if(size>MAX_RBUF_LEN) size=MAX_RBUF_LEN;
    return size;
  }
};

//----------------------------------------------------------
// Словарь
//----------------------------------------------------------

/// Тип элемента словаря
enum { TVOC_INT    = 1,  /// Целое число
       TVOC_FLOAT  = 2,  /// Действительное число
       TVOC_STR    = 3   /// Строка (в кавычках "")
};

/// Элемент словаря
class TVocabItem
{
private:
  void Read(const char *s);
  int tip;
public:
  TVocabItem(const char *s) { Read(s); };
  std::string &GetName(void) { return name; }
  std::string name;
  std::string val;
  void Show(void) { printf("name=%s tip=%d val=[%s]\n", name.c_str(), tip, val.c_str()); };  
};

/**
 * Локальный словарь
 */
class TVocab
{
public:
  std::string name;
  std::vector<TVocabItem> data;
  TVocab(const char *vocname)
  {
    name = vocname;
  }
  TVocab(const char *vocname, const char *filename)
  {
    name = vocname;
    Read(filename);
  }
  TVocab(const char *vocname, FILE *f)
  {
    name = vocname;
    Read(f);
  }
  void Show(void);
  void Read(const char *fname);
  void Read(FILE *f);
  TVocabItem *FindByVal(std::string sval);
  TVocabItem *FindByName(std::string sname);
  TVocabItem *FindByName(int nm);
};

/**
 * Мультисловарь
 */
class TMultiVocab
{
public:
  std::string name;
  std::vector<TVocab> vocs;
  TMultiVocab(const char *vocname)
  {
    name = vocname;
  }
  TMultiVocab(const char *vocname, const char *filename)
  {
    name = vocname;
    Read(filename);
  }
  void Read(const char *fname);
  void Read(FILE *f);
  TVocabItem *FindByVal(std::string vocname, std::string sval);
  TVocabItem *FindByName(std::string vocname, std::string sname);
  TVocabItem *FindByName(std::string vocname, int nm);
  TVocabItem *GetValue(std::string vocname, std::string sname, float &val);
  float GetValue(std::string vocname, std::string sname);
  TVocabItem *GetValue(std::string vocname, std::string sname, std::string &value);
};

/// Глобальный словарь системы
extern TMultiVocab MainVocab;

#endif
