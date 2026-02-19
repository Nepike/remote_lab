/**
 * ER Library
 * Сервисная библиотека
 * \author Robofob
 * \version 1.11
 * \date 07.07.2014
 * \date LP 07.09.2016
*/

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <curses.h>
#include <ros/ros.h>

#include "erlib.h"

#include <msg_ans/ans.h>
#include <msg_rsaction/action.h>
#include <msg_pllcmd/pllcmd.h>

#include "servlib.h"
#include "rcproto/tmucmd.h"
#include "pllcommands.h"
#include "exec_wrapper/exec_wrapper.h"

/// Глобальный словарь системы
TMultiVocab MainVocab("MainVocab");

//----------------------------------------------------------
//
//----------------------------------------------------------

using erl::ErCommand;

struct TColorDef
{
  int id;
  char attr;
  TColorDef(int nid, int nattr)
  {
    id = nid;
    attr = nattr;
  }
};

std::vector<TColorDef> ColorDefList; // Описание вывода сенсоров

void erl::mwSetAttr(WINDOW *w, char attr)
{
  int n = -1;
  for(int i=0;i<ColorDefList.size();i++)
    if(ColorDefList[i].attr==attr)
    {
      n = i + 1;
      break;
    }
  if(n==-1)
  {
    n = ColorDefList.size()+1;
    if(n>=COLORS) n = COLORS-1;
    ColorDefList.push_back(TColorDef(n, attr));
    init_pair(n, (attr>>4) & 0x0F, attr & 0x0F);
  }
  wattron(w, COLOR_PAIR(n));
  wattrset(w, COLOR_PAIR(n));
}

WINDOW *erl::mvInitWindow(int Height, int Width)
{
  initscr();
  start_color();
  WINDOW *w = newwin(Height, Width, 0, 0);
  box(w, 0, 0);
  return w;
}

void erl::mvEndWin(void)
{
  //delwin(w_main);
  //doupdate();
  endwin();
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void erl::AnsShow(const char *title, const msg_ans::ans &msg, int showreg)
{
  printf("::%s (%4d) ", title, (msg.tm % 10000));
  if(showreg & TSHANS_DATA)
  {
    printf("result=%3d, text='%s' ", msg.result, msg.text.c_str());
    int n = msg.data.size();
    printf("data=[ ");
    for(int i=0;i<n;i++)
      printf("%3d ", msg.data[i]);
    printf(" ] ");
  }

  if(showreg & TSHANS_CMD)
  {
    printf("cmd = %2d (%2d, %2d)\n", msg.cmd, msg.arg1, msg.arg2);
  }
  fflush(stdout);
}

void erl::AnsShowPos(const char *title, const msg_ans::ans &msg, WINDOW *w, int y, int x, char attr, int showreg)
{
  if(attr) erl::mwSetAttr(w, attr);
  wmove(w, y, x);
  wclrtoeol(w);
  wprintw(w, "::%s (%4d) ", title, (msg.tm % 10000));

  if(showreg & TSHANS_DATA)
  {
    wprintw(w, "result=%3d, text='%s' ", msg.result, msg.text.c_str());
    int n = msg.data.size();
    wprintw(w, "data=[ ");
    for(int i=0;i<n;i++)
      wprintw(w, "%3d ", msg.data[i]);
    wprintw(w, " ] ");
  }

  if(showreg & TSHANS_CMD)
  {
    wprintw(w, "cmd = %2d (%2d, %2d)", msg.cmd, msg.arg1, msg.arg2);
  }
  wrefresh(w);
}

void erl::ShowSlot(const char *title, const msg_ans::ans &msg, WINDOW *w, int y, int x, char attr)
{
  if(attr) erl::mwSetAttr(w, attr);
  wmove(w, y, x);
  wclrtoeol(w);
  wprintw(w, "::%s (%4d) ", title, (msg.tm % 10000));

  wprintw(w, "result=%3d ", msg.result);
  int n = msg.data.size();
  wprintw(w, "slot=[ ");
  // Выводим не все 640 значений, а меньше (для удобства)
  #define kratn 10 // Кратность вывода
  for(int i=0;i<n;i+=kratn)
    wprintw(w, "%c", msg.data[i]?'*':'.');
  wprintw(w, " ] ");
  wrefresh(w);
}

void erl::printfAttr(WINDOW *w, char attr, const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);

  if(attr) erl::mwSetAttr(w, attr);
  wprintw(w,"%s",s);

  va_end(argptr);
}

void erl::printfPos(WINDOW *w, int y, int x, char attr, const char *fmt, ...)
{
  char s[512];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);

  if(attr) erl::mwSetAttr(w, attr);
  wmove(w, y, x);
  wprintw(w,"%s",s);

  va_end(argptr);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

void ArdSendCommand(const msg_ans::ans &cmdmsg, ros::Publisher &pub, int addr, int cmd, int arg1=0, int arg2=0)
{
  msg_rsaction::action msg;

  msg.team_id = 0;
  msg.agent_id = addr;
  msg.action = cmd;
  msg.arg1 = arg1;
  msg.arg2 = arg2;
  // Разбираемся с data
  
  msg.data.clear();
  for(int i=0;i<cmdmsg.data.size();i++)
    msg.data.push_back(cmdmsg.data[i]);
  pub.publish(msg);
}

void PllSendCommand(ros::Publisher &pub, const char *cmd, int arg1=0, int arg2=0)
{
  msg_pllcmd::pllcmd msg;

  msg.command = cmd;
  msg.arg1 = arg1;
  msg.arg2 = arg2;

  //ROS_INFO("PllSendCommand: cmd %s arg1=%d arg2=%d\n", msg.command.c_str(), msg.arg1, msg.arg2);
  pub.publish(msg);
}

int erl::ExecERCommand(const msg_ans::ans &msg, int addr, ros::Publisher &ard_pub, ros::Publisher &pll_pub, int cmd, int arg1, int arg2)
{
  switch(cmd)
  {
    case ERCMD_STOP:            ArdSendCommand(msg, ard_pub, addr, CMD_STOP); break;
    case ERCMD_GOFWD:           ArdSendCommand(msg, ard_pub, addr, CMD_FWD); break;
    case ERCMD_GOBACK:          ArdSendCommand(msg, ard_pub, addr, CMD_BACK); break;
    case ERCMD_GOLEFT:          ArdSendCommand(msg, ard_pub, addr, CMD_LEFT); break;
    case ERCMD_GORIGHT:         ArdSendCommand(msg, ard_pub, addr, CMD_RIGHT); break;
    case ERCMD_GOFASTLEFT:      ArdSendCommand(msg, ard_pub, addr, CMD_FAST_LEFT); break;
    case ERCMD_GOFASTRIGHT:     ArdSendCommand(msg, ard_pub, addr, CMD_FAST_RIGHT); break;

    case ERCMD_BEEP:            ArdSendCommand(msg, ard_pub, addr, CMD_BEEP); break;
    case ERCMD_BEEP_ON:         ArdSendCommand(msg, ard_pub, addr, CMD_BEEP_ON); break;
    case ERCMD_BEEP_OFF:        ArdSendCommand(msg, ard_pub, addr, CMD_BEEP_OFF); break;

    case ERCMD_GOFWD2:          ArdSendCommand(msg, ard_pub, addr, CMD_FWD2, arg1); break;
    case ERCMD_GOBACK2:         ArdSendCommand(msg, ard_pub, addr, CMD_BACK2, arg1); break;
    case ERCMD_GOLEFT2:         ArdSendCommand(msg, ard_pub, addr, CMD_LEFT2, arg1); break;
    case ERCMD_GORIGHT2:        ArdSendCommand(msg, ard_pub, addr, CMD_RIGHT2, arg1); break;
    case ERCMD_GOFASTLEFT2:     ArdSendCommand(msg, ard_pub, addr, CMD_FAST_LEFT2, arg1); break;
    case ERCMD_GOFASTRIGHT2:    ArdSendCommand(msg, ard_pub, addr, CMD_FAST_RIGHT2, arg1); break;

    case ERCMD_ARD_GET_SENS:    ArdSendCommand(msg, ard_pub, addr, CMD_GET_SENS); break;
    case ERCMD_ARD_GET_ALL_REG: ArdSendCommand(msg, ard_pub, addr, CMD_GET_ALL_REG); break;
    case ERCMD_SET_REG:         ArdSendCommand(msg, ard_pub, addr, CMD_SET_REG, arg1, arg2); break;
    case ERCMD_I2C:             ArdSendCommand(msg, ard_pub, addr, CMD_I2C, arg1, arg2); break;
    case ERCMD_GET_I2C:         ArdSendCommand(msg, ard_pub, addr, CMD_GET_I2C_DATA, arg1, arg2); break;

    // NOTE: KOSTYL
    case ERCMD_SET_SPEED:       ArdSendCommand(msg, ard_pub, addr, CMD_SET_SPEED, arg1, arg2); break;

    case ERCMD_PLL_GET_SENS:    PllSendCommand(pll_pub, PLL_GETALLPOSITIONS, arg1); break;
    default: return 0;
  }
  return 1;
}

int erl::ExecERCommand(const msg_ans::ans &msg, ros::Publisher &ard_pub, ros::Publisher &pll_pub)
{
  return ExecERCommand(msg, msg.id, ard_pub, pll_pub, msg.cmd, msg.arg1, msg.arg2);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

std::vector<erl::TSenDescr> erl::SensorsDescrList; // Описание вывода сенсоров

int getint(char *s, int *n, const char *delimstr, int *res, int hexformat)
{
  l_string v;
  if(!getfragment(s, n, delimstr,v))
    return 0;
  if(hexformat)
    sscanf(v,"%x", res);
  else
    *res = atoi(v);
  return 1;
}

int getbyte(char *s, int *n, const char *delimstr, char *res, int hexformat)
{
  l_string v;
  if(!getfragment(s, n, delimstr,v))
    return 0;
  if(hexformat)
    sscanf(v,"%hhX", res);
  else
    *res = atoi(v);
  return 1;
}

void erl::ReadSensorsDescrList(char *fname)
{
  l_string s, v;
  FILE *f = fopen(fname,"r");
  if(f==NULL)
    error("ReadSensorsDescrList: open file '%s' error", fname);
  erl::TSenDescr sd;
  while(ReadString(f, s))
  {
    int n = 0;

    if(!getfragment(s, &n, " ,;\t", v))
      error("ReadSensorsDescrList: format error (name) at line '%s'", s);
    sd.name = v;

    if(!getCstrfragment(s, &n, v))
      error("ReadSensorsDescrList: format error (prefix) at line '%s'", s);
    sd.prefix = v;

    if(!getCstrfragment(s, &n, v))
      error("ReadSensorsDescrList: format error (vformat) at line '%s'", s);
    sd.vformat = v;

    if(!getCstrfragment(s, &n, v))
      error("ReadSensorsDescrList: format error (suffix) at line '%s'", s);
    sd.suffix = v;

    if(!getint(s, &n, " ,()\t", &sd.y, 0))
      error("ReadSensorsDescrList: format error (y) at line '%s'", s);

    if(!getint(s, &n, " ,()\t", &sd.x, 0))
      error("ReadSensorsDescrList: format error (x) at line '%s'", s);

    if(!getint(s, &n, " ,()\t", &sd.src, 0))
      error("ReadSensorsDescrList: format error (src) at line '%s'", s);
    if(!getint(s, &n, " ,()\t", &sd.offs, 0))
      error("ReadSensorsDescrList: format error (offs) at line '%s'", s);

    if(!getbyte(s, &n, " ,()\t", &sd.attr_pref, 1))
      error("ReadSensorsDescrList: format error (attr_pref) at line '%s'", s);

    if(!getbyte(s, &n, " ,()\t", &sd.attr_val, 1))
      error("ReadSensorsDescrList: format error (attr_val) at line '%s'", s);

    if(!getbyte(s, &n, " ,()\t", &sd.attr_suff, 1))
      error("ReadSensorsDescrList: format error (attr_suff) at line '%s'", s);

    erl::SensorsDescrList.push_back(sd);
  }
  fclose(f);
}

void erl::ShowSensors(WINDOW *w, const msg_senfield::senfield &msg)
{
  msg_ans::ans m;
  for(int i=0; i<erl::SensorsDescrList.size(); i++)
  {
    wmove(w, erl::SensorsDescrList[i].y, erl::SensorsDescrList[i].x);
    erl::printfAttr(w, erl::SensorsDescrList[i].attr_pref, "%s", erl::SensorsDescrList[i].prefix.c_str());

    int dat;
    int n = erl::GetSenVal(i, msg, dat);
    l_string sdat;
    sprintf(sdat, erl::SensorsDescrList[i].vformat.c_str(), dat);
    if(!n)
      for(int n=0;n<strlen(sdat);n++)
        sdat[n] = '-';

    erl::printfAttr(w, erl::SensorsDescrList[i].attr_val, "%s", sdat);
    erl::printfAttr(w, erl::SensorsDescrList[i].attr_suff, "%s", erl::SensorsDescrList[i].suffix.c_str());

  }
  wrefresh(w);
}

int erl::GetSenVal(int sid, const msg_senfield::senfield &msg, int &dat)
{
  if(sid>=erl::SensorsDescrList.size() || sid<0)
    error("GetSenVal: illegal sid (%d)", sid);
  msg_ans::ans m;
  int ssrc = erl::SensorsDescrList[sid].src;
  switch(ssrc)
  {
    case erl::SRC_ARD_DATA:
      m = msg.arddata;
      break;
    case erl::SRC_ARD_REGS:
      m = msg.ardreg;
      break;
    case erl::SRC_ARD_STAT:
      m = msg.ardstatus;
      break;
    case erl::SRC_PLL_DATA:
      m = msg.pll;
      break;
    case erl::SRC_FACEDETECTOR:
      m = msg.face;
      break;
    case erl::SRC_QRCODE:
      m = msg.qrc;
      break;
    default:
      error("GetSenVal: illegal sensor source %d", ssrc);
  }
  int offs = erl::SensorsDescrList[sid].offs;
  dat = 0;
  if(ssrc==erl::SRC_FACEDETECTOR || ssrc==erl::SRC_QRCODE)
  {
    switch(offs)
    {
      case 0: // result
        dat = m.result;
        break;
      case 1: // cmd
        dat = m.cmd;
        break;
      default:
        error("GetSenVal: illegal offs [%d] for sensor [%d]", offs, sid);
    }
  }
  else
  {
    if(offs<m.data.size())
      dat = m.data[offs];
    else
      return 0;
  }
  return 1;
}

int erl::GetSenVal(std::string name, const msg_senfield::senfield &msg, int &dat)
{
  msg_ans::ans m;
  int sid = -1;
  for(int i=0; i<erl::SensorsDescrList.size(); i++)
    if(erl::SensorsDescrList[i].name==name)
    {
      sid = i;
      break; 
    }
  if(sid==-1)
    error("GetSenVal: sensor '%s' not found", name.c_str());
  return erl::GetSenVal(sid, msg, dat);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

int erl::CheckServers(WINDOW *w, int y, int x, const msg_senfield::senfield &msg)
{
  #define ENABLED_WAIT_TIME 5 // Допустимое время ожидания
  int currtime = time(NULL);
  erl::mwSetAttr(w, 0x10);
  wmove(w, y, x);
  wprintw(w, "                                              ");
  wmove(w, y, x);
  // Arduino Data
  int no1 = currtime-msg.arddata.tm>ENABLED_WAIT_TIME;
  if(no1)
    wprintw(w, "[Ard Data is not ready] ");
  // Pololu Data
  int no2 = currtime-msg.pll.tm>ENABLED_WAIT_TIME;
  if(no2)
    wprintw(w,"[Pll Data is not ready]");
  wrefresh(w);
  return no1 | (no2<<1);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

ExecutionNode say_process, read_text_process, cmd_process;

void erl::Say(const char *msg)
{
  l_string s;
  sprintf(s, "echo \"%s\" | festival --tts --language russian", msg);
  if(say_process.hasStarted())
    if(!say_process.hasFinished()) return;
  say_process.setCommand(s);
  say_process.startExecution(false);
  //system (s);
}

void erl::ReadText(const char *filename)
{
  l_string s;
  sprintf(s, "cat %s | festival --tts --language russian", filename);
  if(read_text_process.hasStarted())
    if(!read_text_process.hasFinished()) return;
  read_text_process.setCommand(s);
  read_text_process.startExecution(false);
  //system (s);
}

void erl::RunCommand(const char *command)
{
  if(cmd_process.hasStarted())
    if(!cmd_process.hasFinished()) return;
  cmd_process.setCommand(command);
  cmd_process.startExecution(false);
  //system (command);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

int erl::kbhit(void)
{
  int ch = getch();
  if (ch != ERR)
  {
    ungetch(ch);
    return 1;
  }
  return 0;
}

void erl::Alarm(WINDOW *w, const char *msg, int iterations)
{
  int c;
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  int cnt = 0;
  while(ros::ok() && cnt<iterations)
  {
    erl::printfPos(w, 0, 0, 0x14, "%s", msg);
    wrefresh(w);
    erl::Say(msg);
    if(erl::kbhit()) c = getch();
    if(c==27) break;
    cnt++;
  }
  nodelay(stdscr, FALSE);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

int TRBuffer::GetAvr(void)
{
  // sic ???
  float S = 0;
  for(int i=0;i<pos;i++)
    S+=buf[i];
  for(int i=pos;i<size;i++)
    S+=buf[i];
  S = S/size;
  return (int)S;
}

//----------------------------------------------------------
//
//----------------------------------------------------------

int lextype(char *lex)
{
  int np = 0;
  for(int i=0;i<(int)strlen(lex);i++)
    if(lex[i] == '.') np++;
    else
    if(lex[0] == '-');
    else
      if((unsigned char)lex[i] < (unsigned char)'0' || (unsigned char)lex[i] > (unsigned char)'9') return -1;
  if(np == 0) return TVOC_INT;
  if(np == 1) return TVOC_FLOAT;
  return -1;
}

/// Элемент словаря
void TVocabItem::Read(const char *s)
{
  l_string v;
  int n = 0;
  if(!getfragment(s, &n, " \t,", v))
    error("TVocabItem::Read format error 1 (name) at line '%s'", s);
  name = v;

  // Пытаемся определить, что там идет дальше
  int predn = n;
  if(!getfragment(s, &n, " \t,", v))
    error("TVocabItem::Read format error 2 at line '%s'", s);
  n = predn;

  if(v[0]=='"') // Строка
  {
    tip=TVOC_STR;
    if(!getCstrfragment(s, &n, v))
      error("TVocabItem::Read format error 3 (str val) at line '%s'", s);
    val = v;
    return;
  }
  // Число
  tip = lextype(v);
  if(tip == -1)
    error("TVocabItem::Read format error 4 (val) at line '%s'", s);
  val = v;
}

/// Словарь
#define VOC_END_MARKER "<!>"

void TVocab::Read(FILE *f)
{
  l_string s, v;
  while(ReadString(f, s))
  {
    if(strcmp(s,VOC_END_MARKER)==0) break;
    TVocabItem itm(s);
    data.push_back(itm);
  }
  Show();
}

void TVocab::Read(const char *fname)
{
  printf("\nStart at [%s] vocab file\n", fname);
  FILE *f = fopen(fname,"r");
  if(f==NULL)
    error("ReadVocab: open file '%s' error", fname);
  Read(f);
  fclose(f);
}

void TVocab::Show(void)
{
  printf("\n--------------------");
  printf("\nVocab [%s]\n", name.c_str());
  for(int i=0;i<data.size();i++)
    data[i].Show();
  fflush(stdout);
}

TVocabItem *TVocab::FindByVal(std::string sval)
{
  for(int i=0;i<data.size();i++)
    if(data[i].val==sval)
      return &data[i];
  return NULL;
}

TVocabItem *TVocab::FindByName(std::string sname)
{
  for(int i=0;i<data.size();i++)
    if(data[i].name==sname)
      return &data[i];
  return NULL;
}

TVocabItem *TVocab::FindByName(int nm)
{
  l_string snm;
  sprintf(snm, "%d", nm);
  return FindByName(snm);
}

/// Мультисловарь
void TMultiVocab::Read(FILE *f)
{
  l_string s;
  while(ReadString(f, s))
  {
    TVocab v(s, f); 
    vocs.push_back(v);
  }
}

void TMultiVocab::Read(const char *fname)
{
  FILE *f = fopen(fname,"r");
  if(f==NULL)
    error("ReadVocab: open file '%s' error", fname);
  Read(f);
  fclose(f);
}

TVocabItem *TMultiVocab::FindByVal(std::string vocname, std::string sval)
{
  // Ищем словарь
  for(int i=0;i<vocs.size();i++)
    if(vocs[i].name==vocname)
      return vocs[i].FindByVal(sval);
  return NULL;
}

TVocabItem *TMultiVocab::FindByName(std::string vocname, std::string sname)
{
  // Ищем словарь
  for(int i=0;i<vocs.size();i++)
    if(vocs[i].name==vocname)
      return vocs[i].FindByName(sname);
  return NULL;
}

TVocabItem *TMultiVocab::FindByName(std::string vocname, int nm)
{
  l_string snm;
  sprintf(snm, "%d", nm);
  return FindByName(vocname, snm);
}

TVocabItem *TMultiVocab::GetValue(std::string vocname, std::string sname, float &val)
{
  float r;
  val = 0;
  TVocabItem *itm = FindByName(vocname, sname);
  if(!itm) return NULL;
  if(sscanf(itm->val.c_str(), "%f", &r)!=1) return NULL;
  val = r;
  return itm;
}

float TMultiVocab::GetValue(std::string vocname, std::string sname)
{
  float r;
  TVocabItem *itm = FindByName(vocname, sname);
  if(!itm) // return 0;
    error("TMultiVocab::GetValue: '%s.%s' not found", vocname.c_str(), sname.c_str());
  if(sscanf(itm->val.c_str(), "%f", &r)!=1) // return 0;
    error("TMultiVocab::GetValue: format error at '%s.%s'", vocname.c_str(), sname.c_str());
  return r;
}

TVocabItem *TMultiVocab::GetValue(std::string vocname, std::string sname, std::string &value)
{
  value = "";
  TVocabItem *itm = FindByName(vocname, sname);
  if(!itm)
    error("TMultiVocab::GetValue: '%s.%s' not found", vocname.c_str(), sname.c_str());
  value = itm->val;
  return itm;
}
