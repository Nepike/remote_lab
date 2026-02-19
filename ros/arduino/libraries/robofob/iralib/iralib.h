/*
 * Псевдоаналоговый канал связи
 * 
 * V 1.04
 * 19.04.2017/06.06.2017
 * LP 06.02.2023
 *
 * Генерация и измерение длин пачек импульсов
 *
 * Рабочий таймер Timer1
 *    - отвечает за генерацию меандра с частотой TSOP (36, 56кГц).
 *    - измеряет приходящий с TSOP сигнал
 * Генерирующий таймер Timer2
 *    - "дергает" ногу outp (излучатель)
 * 
 * Материалы: https://www.instructables.com/id/Arduino-Timer-Interrupts/
 * 
 */
#ifndef _IRALIB_H_
#define _IRALIB_H_

#include <Arduino.h>

namespace iralib
{
  const int NTS = 4;        // Количество TSOP
  const byte MAX_PNUM = 15; // Максимальный код
  void Init(void);
  void Evaluate(byte result[]);
  // Установка параметров генератора в соответствии с требуемым кодом
  void SetPNum(byte N);
}

#endif
