/**
 * Обратная кинематическая задача
 * \version 1.15
 * Автор: Карпов В.Э.
 * \date: 23 декабря 2006
 * Дата последней модификации: 14 января 2008
 * LP 04.06.2014
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include "maniplib.h"

using namespace ManipLib;

//----------------------------------------------------------
// Вспомогательные сервисные функции
//----------------------------------------------------------
#define MAX_STR 512
static int V_scanf(FILE *point, char *msg, int len)
{
  msg[0] = 0;
  if(fgets(msg, len, point) == NULL)
  {
    if(msg[0]) return 1;
    return 0;
  }
  if(msg[strlen(msg)-1] == 10) msg[strlen(msg)-1] = 0;
  return 1;
}

/// удаление начальных и хвостовых пробелов из строки
static void trim(char *s)
{
  char *v = s;
  int n=strlen(v)-1;
  while((unsigned char)v[n]<=32 && n>0)
    { v[n]=0; n--; }

  while(strlen(v)>0 && ((unsigned char)v[0]<=32))
      v++;
  strcpy(s,v);
}

/// Определение номера позиции символа 'с' в строке s,
/// возвращает номер или -1.
static int find_char(char c, const char *s)
{
  int i;
  for(i=0;*s;i++) if(*s++==c) return i;
  return -1;
}

/// возвращает первую строку, не являющуюся комментарием
static int SkipRemarkLine(FILE *f, char *s)
{
  int k, ok;
  do
  {
    if(!V_scanf(f,s,MAX_STR))
    {
      s[0] = 0;
      return 0;
    }
    trim(s);
    k = find_char(';',s);
    if(k==0) s[k] = 0;
    ok = (s[0]!=0);
  } while (!ok);
  return 1;
}

void ManipLib::readln(FILE *f, char *s)
{
  if(!SkipRemarkLine(f, s))
    error("INI format error 1");
}

int ManipLib::readln(FILE *f)
{
  char s[MAX_STR];
  if(!SkipRemarkLine(f, s))
    error("INI format error 2");
  int n = atoi(s);
  return n;
}

void ManipLib::error(const char *fmt, ...)
{
  char s[MAX_STR];
  va_list argptr;
  va_start(argptr, fmt);
  vsprintf(s, fmt, argptr);
  va_end(argptr);
  puts(s);
  exit(1);
}

//----------------------------------------------------------
// Вектор
//----------------------------------------------------------

TVect::TVect(int n)
{
  dim = n;
  for(int i=0;i<MAXCHAINNUM;i++)
    data[i]=0;
}

void TVect::operator=(TVect & V)
{
  for(int i=0;i<dim;i++)
    data[i] = V.data[i];
  dim = V.dim;
}

void TVect::operator=(int n)
{
  for(int i=0;i<dim;i++)
    data[i] = n;
}

int TVect::operator!=(TVect & V)
{
  for(int i=0;i<dim;i++)
    if(data[i] != V.data[i]) return 1;
  return 0;
}

int TVect::operator==(TVect & V)
{
  for(int i=0;i<dim;i++)
    if(data[i] != V.data[i]) return 0;
  return 1;
}

int TVect::getval(int num, int *val)
{
  if (dim > num);
  else return 1;

  *val = data[num];
  return 0;
}

int TVect::setval(int val, int i)
{
  if(dim < i)
    return -1;

  data[i] = val;
  return 0;
}

void TVect::print(const char *s)
{
  printf("%s",s);
  for(int i=0;i<dim;i++)
    printf("%d ",data[i]);
}

//----------------------------------------------------------
//
//----------------------------------------------------------

#define StartChain    1   //Номер начального звена

//----------------------------------------------------------
// Звенья
//----------------------------------------------------------
TChain::TChain(FILE *f)
{
  /// <номер порта Pololu> <базовый угол> <нач.положение> <длина звена> <углы поворота a0 a1> <коэфф.>
  char s[MAX_STR];
  readln(f,s);
  sscanf(s,"%d %d %d %d %d %d %f", &lineNum, &BaseA, &StartA, &L, &aMin, &aMax, &k);

  x0 = y0 = z0 = a0 = 0;
  a = StartA;
}

/// Контроль нижнего положения
int ManipLib::tmCheckYPos = 0;  /// Включить проверку дополнительного условия
                                /// (режим "не ниже целевой точки")
int ManipLib::tmMinY = 0;       /// Минимальное положение по вертикали
                                /// (координата, ниже которой нельзя опускать манипулятор)
int ManipLib::tmdDS = 10;       /// Глубина памяти размер окрестности поиска)

//----------------------------------------------------------
// Манипулятор
//----------------------------------------------------------
TManip::TManip(FILE *f)
{
  ChainNum = readln(f);
  for(int i=0;i<ChainNum;i++)
    Chain[i] = new TChain(f);
}

void TManip::SetStat(TVect a)
{
  for(int i=StartChain;i<ChainNum;i++)
    Chain[i]->setang(a[i]);

  for(int i=StartChain+1;i<ChainNum;i++)
  {
    int x1 = Chain[i-1]->cx();
    int y1 = Chain[i-1]->cy();
    Chain[i]->SetPos0(x1, y1, 0, Chain[i-1]->getAbsAngle());
  }
}

void TManip::SetStat(void)
{
  for(int i=StartChain+1;i<ChainNum;i++)
  {
    int x1 = Chain[i-1]->cx();
    int y1 = Chain[i-1]->cy();
    Chain[i]->SetPos0(x1, y1, 0, Chain[i-1]->getAbsAngle());
  }
}

double TManip::Dist(int gx, int gy, TVect a)
{
  SetStat(a);
  int x2 = Chain[ChainNum-1]->cx();
  int y2 = Chain[ChainNum-1]->cy();
  double d = sqrt(pow(gx-x2,2)+pow(gy-y2,2));
  return d;
}

double TManip::Dist(int gx, int gy)
{
  TVect a(MAXCHAINNUM);
  for(int i=0;i<ChainNum;i++)
    a[i] = Chain[i]->getang();
  double d = Dist(gx, gy, a);
  return d;
}

int TManip::GetAngles(TVect &V)
{
  for(int i=0;i<ChainNum;i++)
    V[i] = Chain[i]->getang();
  return ChainNum;
}

int TManip::getAbsAngle(int chnum)
{
  int a = 0;
  for(int i=StartChain+1;i<=chnum;i++)
    a+= Chain[i]->getAbsAngle();
  return a;
}

int TManip::Search3DPos(int x, int y, int z)
{
  // Угол поворота основания
  double a = (x==0 && y==0)?0:atan2(y,x);
  int a0 = rad2grad(a);
  if(a0>Chain[0]->getMaxAng()) a0 = Chain[0]->getMaxAng();
  if(a0<Chain[0]->getMinAng()) a0 = Chain[0]->getMinAng();
  Chain[0]->setang(a0);

  double rz = sqrt(pow(x,2)+pow(y,2));
  int _x = rz;
  int _y = z - Chain[0]->getlen(); //Учитываем длину нулевого звена
  int n = SearchPos(_x, _y);
  return n;
}

int TManip::MoveTo(int gx, int gy, int gz, TVect &V, int *x, int *y, int *z, double *dist)
{
  int n = Search3DPos(gx, gy, gz);
  GetAngles(V);
  int rx, ry, rz;
  GetEndPos(rx, ry, rz);
  *dist = distance(gx,gy,gz, rx, ry, rz);
  *x = rx;
  *y = ry;
  *z = rz;
  return n;
}

void TManip::GetEndPos(int &x, int &y, int &z)
{
  SetStat();
  double r = cx();
  double rz = cy();
  double a = grad2rad(Chain[0]->getang());
  double rx = r*cos(a);
  double ry = r*sin(a);

  x = rx;
  y = ry;
  z = rz+Chain[0]->getlen();
}

void TManip::GetEndPos(int chnum, int &x, int &y, int &z)
{
  SetStat();
  if(chnum==0)
  {
    x = y = 0;
    z = Chain[0]->getlen();
    return;
  }
  double r = cx(chnum);
  double rz = cy(chnum);
  double a = grad2rad(Chain[0]->getang());
  double rx = r*cos(a);
  double ry = r*sin(a);

  x = rx;
  y = ry;
  z = rz+Chain[0]->getlen();
}

int TManip::SearchPos(int gx, int gy)
{
  #define MAXCNT 1000
  int cnt=0;
  TVect a(MAXCHAINNUM);
  TVect olda(MAXCHAINNUM);
  for(int i=1;i<ChainNum;i++)
    a[i] = Chain[i]->getang();

  TVect ZN(ChainNum);    // Массив текущих значений ZN[N]
  TVect L0(ChainNum);    // Массив начальных значений L0[N]
  TVect LK(ChainNum);    // Массив конечных значений LK[N]
  TVect STEP(ChainNum);  // Массив шагов циклов STEP[N]
  TVect ta(MAXCHAINNUM);
  TVect ma(MAXCHAINNUM);

  int changed;
  do
  {
    cnt++;
    olda = a;
    SetStat(a);

    double md = Dist(gx, gy, a);
    ma = a;

    //----------------------------------------------------
    // N вложенных циклов
    //----------------------------------------------------
    int I;
    int N=ChainNum;

    L0 = -tmdDS;
    LK =  tmdDS;
    ZN = L0;
    STEP = 1;

A:;
    I=StartChain; // Т.к. звено 0 пропускаем
    //------------------------------------
    // Здесь выполняются необходимые действия над ZN[i]
    //------------------------------------
    // Выбираем лучший
    for(int i=StartChain;i<ChainNum;i++)
      ta[i] = a[i]+ZN[i];

    for(int i=StartChain;i<ChainNum;i++)
    {
      if(ta[i]>Chain[i]->getMaxAng()) ta[i] = Chain[i]->getMaxAng();
      if(ta[i]<Chain[i]->getMinAng()) ta[i] = Chain[i]->getMinAng();
    }
    double d = Dist(gx, gy, ta);

    // Дополнительные условия
    /* ***  Не нужно этих глупостей
    int ccy = cy();
    if(tmCheckYPos && ccy<gy)
      goto B; // or continue
    if(ccy<tmMinY)
      goto B; // or continue
    *** */
    if(d<md)
    {
      md = d;
      ma = ta;
    }
    //-------------------------------------
    // Конец действий
    //-------------------------------------
B:;
    if(ZN[I]+1>=LK[I])
    {
      if(I>=N-1) goto C;
      ZN[I] = L0[I];
      I=I+1;
      goto B;
    }
    ZN[I]=ZN[I]+STEP[I];
    goto A;
C:;
    //------------------------------------
    // Конец циклов
    //------------------------------------

    a = ma;
    SetStat(a);
    changed = (olda != a);

  } while(changed && cnt<MAXCNT);
  return cnt;
}
