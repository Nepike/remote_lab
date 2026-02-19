/*
 * Шаговый двигатель
 * 4 концевых датчика
 * 
 * Arduino Mega
 * Author: Valery Karpov
 *
 * Version 2.51
 *
 * 21.06.2023, 20.12.2023, 04.04.2024
 * LP 24.06.2024
 *   
*/

#ifndef _ONDSTEPM_
#define _ONDSTEPM_

namespace ondstepm
{

// Концевые датчики
const int8_t NUM_LIM_SW = 4;

extern const int8_t PIN_LIM_SW[NUM_LIM_SW];

extern int8_t LIM_SW[NUM_LIM_SW];

byte LIM_SW_STAT = 0;

const unsigned int IMP_WIDTH = 2;  // Ширина импульса step для ШД, микросекунды

const long T1Period = 1000L;       // Период прерывания таймера, микросекунды.

//------------------------------------------------------------------------------
// Драйвер ШД
//------------------------------------------------------------------------------
struct TStepMotor
{
  TStepMotor(int8_t _p_en, int8_t _p_dir, int8_t _p_step)
  {
    pin_enable = _p_en;
    pin_dir = _p_dir;
    pin_step = _p_step;

    StepCounter = 0;  // Текущее количество шагов (импульсов) от текущей команды
    T1Divider = 100;  // Делитель частоты

    stmEnable = false;
    stmDir = 0;
    stmFreq = 0;
    stmStepNum = 0;
    stmSwNum = -1;   // Номер концевика
    stmSwVal = -1;   // Значение концевого датчика для условия останова вращения

    _n_timer = 0;
    GCNT = 0;
    ClearEojSw();
    _init_ports();
    DisableSTM();
  }
  void EnableSTM(void) { stmEnable = true; digitalWrite(pin_enable, LOW); }
  void DisableSTM(void) { stmEnable = false; digitalWrite(pin_enable, HIGH); digitalWrite(pin_step, LOW); digitalWrite(pin_dir, LOW);}
  bool STMEnabled(void) { return digitalRead(pin_enable)==0; }
  void ClearEojSw(void) { for(byte i=0;i<NUM_LIM_SW;i++) eoj_sw[i] = -1; }

  // Преобразование внешних управляющих параметров во внутренние
  void Ctl2MovementParams(void)
  {
    stmEnable = true;

    if(stmFreq>0) T1Divider = 1000000L/(stmFreq * T1Period);
    else stmEnable = false;

    if(!stmDir) stmEnable = false;
    if(stmEnable) EnableSTM(); else DisableSTM();

    if(stmSwNum>=0 && stmSwNum<NUM_LIM_SW)
      eoj_sw[stmSwNum] = stmSwVal;

    digitalWrite(pin_dir, (stmDir>0));
    StepCounter = 0;
  }
  void Exec(void)
  {
    /*
    bool curr_enabled = STMEnabled();
    if(stmEnable && !curr_enabled) EnableSTM();
    if(!stmEnable && curr_enabled) DisableSTM();
    if(!stmEnable) return;
     */  
    
    _n_timer++;
    if(_n_timer>=T1Divider) _n_timer = 0;
    else return;

    // Условие завершения вращения
    bool stop_cond = false;
    for(byte i=0;i<NUM_LIM_SW;i++)
      if (eoj_sw[i] == LIM_SW[i]) stop_cond = true;
    if((stmStepNum>0 && StepCounter>stmStepNum) || stop_cond)
    {
      ClearEojSw();
      stmEnable = false;
      DisableSTM();
      return;
    }
    // Генерируем импульс
    StepCounter++;
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(IMP_WIDTH);
    digitalWrite(pin_step, LOW);
    // Счетчик
    GCNT+= stmDir;
  }

  // Ноги
  int8_t pin_enable;  // Разрешение работы драйверу ШД
  int8_t pin_dir;     // Направление вращения двигателя
  int8_t pin_step;    // Импульс шага двигателя

  // Счетчики
  long StepCounter;  // Текущее количество шагов (импульсов)
  int T1Divider;     // Делитель частоты
  // Внешние управляющие параметры
  bool stmEnable;    // Флаг разрешения движения
  int stmDir;        // Направление: +1, -1 - туда/сюда, 0 - стоим на месте
  int stmFreq;       // Частота импульсов, Гц
  int stmStepNum;    // Количество шагов (если <0, то идет непрерывное движение)

  int8_t stmSwNum;   // Номер концевика
  int8_t stmSwVal;   // Значение концевого датчика для условия останова вращения
  // Условия завершения работы: 0, 1 - по значению датчика LIM_SW_x == 0, 1 соответственно, -1 - не проверять условие
  int8_t eoj_sw[NUM_LIM_SW];

  long GCNT;         // Глобальный счетчик шагов

private:
  int _n_timer;      // Внутренний счетчик
  void _init_ports(void)
  {
    pinMode(pin_enable, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_step, OUTPUT);
  }

};

};

#endif
