/**
 *
 * Ring Buffer Template
 *
 * V 1.01
 * 20.11.2017
 * LP 20.01.2018
 */

#ifndef _RBUFF_LIB_H_

#define _RBUFF_LIB_H_

#include <Arduino.h>

// BUF_LEN - длина буфера
template <class T, size_t BUF_LEN> class TRBuffer
{
private:
  byte cp;
  T buff[BUF_LEN];

public:
  TRBuffer(void);
  T GetVal(void) const;   // Получить среднее
  void Add(T n);          // Добавить значение
  void SetAll(T n);       // Установить все значения

  T GetMax(void) const;   // Максимальное значение в буфере
  T GetMin(void) const;   // Минимальное значение в буфере
  T GetD(void) const;     // Разница между максимальным и минимальным значениями
};

template <class T, size_t BUF_LEN> TRBuffer<T, BUF_LEN>::TRBuffer(void)
{
  for(byte i=0;i<BUF_LEN;i++)
    buff[i] = 0;
   cp = 0;
}

template <class T, size_t BUF_LEN> void TRBuffer<T, BUF_LEN>::SetAll(T n)
{
  for(byte i=0;i<BUF_LEN;i++)
    buff[i] = n;
}

template <class T, size_t BUF_LEN> T TRBuffer<T, BUF_LEN>::GetVal(void) const
{
  float sum = 0;
  for(byte i=0;i<BUF_LEN;i++)
    sum+=buff[i];
  return (T)(sum/BUF_LEN);
}

template <class T, size_t BUF_LEN> void TRBuffer<T, BUF_LEN>::Add(T n) 
{
  buff[cp] = n;
  cp++;
  if(cp>=BUF_LEN) cp = 0;
}

template <class T, size_t BUF_LEN> T TRBuffer<T, BUF_LEN>::GetMax(void) const
{
  T cmax = buff[0];
  for(byte i=0;i<BUF_LEN;i++)
    if(buff[i]>cmax) cmax = buff[i];
  return cmax;
}

template <class T, size_t BUF_LEN> T TRBuffer<T, BUF_LEN>::GetMin(void) const
{
  T cmin = buff[0];
  for(byte i=0;i<BUF_LEN;i++)
    if(buff[i]<cmin) cmin = buff[i];
  return cmin;
}

template <class T, size_t BUF_LEN> T TRBuffer<T, BUF_LEN>::GetD(void) const
{
  T cmin = buff[0];
  T cmax = buff[0];
  for(byte i=0;i<BUF_LEN;i++)
  {
    if(buff[i]<cmin) cmin = buff[i];
    if(buff[i]>cmax) cmax = buff[i];
  }
  return (cmax-cmin);
}

#endif
