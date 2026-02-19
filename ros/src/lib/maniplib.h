/**
 * Обратная кинематическая задача
 * \version 1.15
 * Автор: Карпов В.Э.
 * \date: 23 декабря 2006
 * Дата последней модификации: 14 января 2008
 * LP 04.06.2014
*/

#ifndef _MANIP_LIB_H_
#define _MANIP_LIB_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>


namespace ManipLib
{
  /// Вспомогательные сервисные функции
  void error(const char *fmt, ...);
  void readln(FILE *f, char *s);
  int readln(FILE *f);

  /// Математика
  inline double grad2rad(double grad) { return grad*M_PI/180.0; }
  inline double rad2grad(double rad) { return rad*180.0/M_PI; }
  inline double distance(int x1, int y1, int z1, int x2, int y2, int z2)
  { return sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2)); }

  #define MAXCHAINNUM 32

  /*
   * Вектор
   */
  class TVect
  {
    private:
      int dim;
      int data[MAXCHAINNUM];
    public:
      TVect(int n);
      void print(const char *s);
      void operator=(TVect & V);
      void operator=(int);
      int & operator[](int idx) { return data[idx]; };
      int operator!=(TVect & V);
      int operator==(TVect & V);
      int getval(int num, int *val); // Added new method
      int setval(int val, int i);    // Added new method
      int len(void) { return dim; }
  };

  /*
   * Звенья
   */

  class TChain
  {
    private:
      int a;            /// Текущий угол поворота
      int L;            /// Длина звена
      int StartA;       /// Стартовый угол (от -60 до +60, град.)
      int BaseA;        /// Базовый угол звена. Определяется конструкцией. Инвариант.
      int x0, y0, z0;   /// Координаты начала звена
      int a0;           /// Начальный угол поворота (база)
      int aMin, aMax;   /// Пределы поворота, град.
      float k;          /// Коэффициент поправки
      int lineNum;      /// Номер line на Pololu Maestro
    public:
      TChain(FILE *f);
      int getMaxAng(void) { return aMax; }
      int getMinAng(void) { return aMin; }
      int getLineNum (void) { return lineNum; }
      int getBaseA(void) { return BaseA; }
      int getStartA(void) { return StartA; }
      float getCoeff(void) { return k; }
      int getang(void) { return a; }
      int getlen(void) { return L; }
      int geta0(void) { return a0; }
      int getAbsAngle(void) { return a+a0+BaseA; } /// Абсолютный угол поворота звена
      int getx0(void) { return x0; }
      int gety0(void) { return y0; }

      void setang(int na) /// Установить угол
      { a = na; }

      void SetPos0(int nx0, int ny0, int nz0, int na0)
      {
        x0 = nx0;
        y0 = ny0;
        z0 = nz0;
        a0 = na0;
      }
      int cx(void)
      {
        int x=x0+L*cos(grad2rad(a+a0+BaseA));
        return x;
      }
      int cy(void)
      {
        int y=y0+L*sin(grad2rad(a+a0+BaseA));
        return y;
      }
  };

  //----------------------------------------------------------
  //
  //----------------------------------------------------------

  // Контроль нижнего положения
  extern int tmCheckYPos; /// Включить проверку дополнительного условия
                          /// (режим "не ниже целевой точки")
  extern int tmMinY;      /// Минимальное положение по вертикали
                          /// (координата, ниже которой нельзя опускать манипулятор)
  extern int tmdDS;       /// Глубина памяти (размер окрестности поиска)

  class TManip
  {
    protected:
      int ChainNum;               /// Количество звеньев
      TChain *Chain[MAXCHAINNUM]; /// Звенья
      double Dist(int gx, int gy, TVect a);
      double Dist(int gx, int gy);
      void SetStat(TVect a);
      void SetStat(void);
      int cx(void) { return Chain[ChainNum-1]->cx(); }
      int cy(void) { return Chain[ChainNum-1]->cy(); }
      int cx(int chmum) { return Chain[chmum]->cx(); }
      int cy(int chmum) { return Chain[chmum]->cy(); }
    public:
      TManip() {};
      TManip(FILE *f);
      /**
        Возвращает количество итераций
        gx, gy, gz - целевая точка

        1. Дана точка (x,y,z)
        2. a := arctan(y/x)
        3. Поворот звена Z0 на угол a: Z0a
        4. rz := sqrt(x^2+y^2)
        5. Определить (x',y'): x' = rz, y'=z
        Работаем в плоскости XZ
      */
      int Search3DPos(int gx, int gy, int gz);
      int SearchPos(int gx, int gy);
      /// Возвращает количество итераций
      /// gx, gy, gz - целевая точка
      /// V - вектор углов поворотов звеньев
      /// x, y, z - результирующая точка (куда фактически пришли)
      /// dist - расстояние между целевой и фактическими точками
      int MoveTo(int gx, int gy, int gz, TVect &V, int *x, int *y, int *z, double *dist);

      int GetChainNum(void) { return ChainNum; }
      void GetEndPos(int &x, int &y, int &z);
      void GetEndPos(int chnum, int &x, int &y, int &z);
      int GetAngles(TVect &V);
      int getAbsAngle(int chnum); /// Абсолютный угол поворота звена

      int getMaxAng(int chmum) { return Chain[chmum]->getMaxAng(); }
      int getMinAng(int chmum) { return Chain[chmum]->getMinAng(); }

      int getLineNum(int chmum) { return Chain[chmum]->getLineNum(); }
      float getCoeff(int chmum) { return Chain[chmum]->getCoeff(); }
      int getBaseA(int chmum) { return Chain[chmum]->getBaseA(); }

      int geta0(int chmum) { return Chain[chmum]->geta0(); }
      int getStartA(int chmum) { return Chain[chmum]->getStartA(); }
  };
}

#endif
