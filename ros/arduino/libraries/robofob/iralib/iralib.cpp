/*
 * Псевдоаналоговый канал связи
 * 
 * V 1.04
 * 19.04.2017/06.06.2017
 * LP 06.02.2023
 *
 * Генерация и измерение длин пачек импульсов
 * Кодируемая последовательность - от 1 до 10 (15)
 * 0 - останов генерации
 *
 * Рабочий таймер Timer1
 *    - отвечает за генерацию меандра с частотой TSOP (36, 56кГц).
 *    - измеряет приходящий с TSOP сигнал
 * Генерирующий таймер Timer2
 *    - "дергает" ногу PIN_OUT (излучатель)
 * 
 * Входы подключения TSOP: 8, 9, 10, 11
 * Выход излучателя PIN_OUT: 3
 *
 * Материалы: https://www.instructables.com/id/Arduino-Timer-Interrupts/
 * 
 */

#include <Arduino.h>
#include "iralib.h"

using namespace iralib;

// Излучатель
#define TIMER_PWM_PIN        3  /* Arduino Duemilanove, Diecimila, LilyPad, etc */
#define PIN_OUT TIMER_PWM_PIN

#define PIN_DEBUG 4 // Отладочный выход

// Входы подключения TSOP
byte TSOP_PIN[NTS] = {8, 9, 10, 11};

//----------------------------------------------------------
// Параметры фильтрации и инерционности
//----------------------------------------------------------
#define MSPEED    2 //  2 Максимально допустимая скорость изменения значения измеряемого сигнала
#define LAG_TIME  4 // 4 10 Инерционность датчика

#define BUF_LEN   4 //  4 Размер кольцевого буфера

//----------------------------------------------------------
// Исходные данные
//----------------------------------------------------------

#define FTSOP 36000L  // 36000 F, несущая частота TSOP, Гц. Таймер 2 работает с частотой FTSOP*2

#define WTFREQ (FTSOP/4L) // Частота рабочего таймера
// Это означает, что на единицу кода IR_CODE приходится 8 выходных импульсов

byte WTITER_RAT = (int)WTFREQ/(int)100;

// 2 границы: для паузы между пачками [0] пачки и длины самой пачки [1] 
byte TWDC[2] = {0,0};

// Рабочий счетчик
byte Tcnt = 0;

enum { ST_COUNT, ST_START };

// Первичные рецепторы 
struct TTSOP
{
  byte stat;
  byte spred;
  byte cnt;
  byte res;
  byte t;
  byte pred_t;
  TTSOP(void)
  {
    stat = ST_START;
    spred = 1; // 1
    cnt = res = t = pred_t = 0;
    t = LAG_TIME+1;
  }
};

TTSOP tm[NTS];

//-----------------------------------------------------------------------

struct TRingBuffer
{
  byte buff[BUF_LEN];
  byte GetVal(void);
  void Add(byte n);
  void SetAll(byte n);
  short int Speed(void);
  byte cp;
  TRingBuffer(void);
};

TRingBuffer::TRingBuffer(void)
{
  memset(buff, 0, sizeof(buff));
  cp = 0;
}

void TRingBuffer::SetAll(byte n)
{
  for(byte i=0;i<BUF_LEN;i++)
    buff[i] = n;
}

byte TRingBuffer::GetVal(void)
{
  int sum = 0;
  for(byte i=0;i<BUF_LEN;i++)
    sum+=buff[i];
  return (byte)(sum/BUF_LEN);
}

void TRingBuffer::Add(byte n)
{
  buff[cp] = n;
  cp++;
  if(cp>=BUF_LEN) cp = 0;
}

short int TRingBuffer::Speed(void)
{
  byte n0 = cp;
  short int n1 = cp - 1;
  if(n1<0) n1 = BUF_LEN-1;
  short int speed = (short int)buff[n0]-(short int)buff[n1];
  return speed;
}

TRingBuffer rb[NTS];

// Установка параметров генератора в соответствии с требуемым кодом
void iralib::SetPNum(byte N)
{
  if(N>MAX_PNUM) N = MAX_PNUM;
  TWDC[0] = N; //3
  TWDC[1] = N;  
}

/*
 * 
 */

// Частота генератора
#ifdef F_CPU
#define SYSCLOCK F_CPU     // main Arduino clock
#else
#define SYSCLOCK 16000000  // main Arduino clock
#endif

#define TIMER_RESET
#define TIMER_ENABLE_PWM     (TCCR2A |= _BV(COM2B1))
#define TIMER_DISABLE_PWM    (TCCR2A &= ~(_BV(COM2B1)))
#define TIMER_CONFIG_KHZ(val) ({ \
  const uint8_t pwmval = SYSCLOCK / 2000 / (val); \
  TCCR2A = _BV(WGM20); \
  TCCR2B = _BV(WGM22) | _BV(CS20); \
  OCR2A = pwmval; \
  OCR2B = pwmval / 3; \
})


// timer1. Рабочий таймер
ISR(TIMER1_COMPA_vect)
{
  static byte qtCond = 0;   // Флаг состояния для генерации сигнала маяка: либо пауза, либо пачка
  static byte cntt2gen = 0; // Счетчик для генерации пачки
  static byte tk=0;

#define NUM_IMP 3
#define NUM_EMP 3
  //
  // Генерация пачек импульсов
  //
  // В зависимости от состояния qtCond определяется либо длительность пачки, либо задержка.
  cntt2gen++;
  if(cntt2gen>TWDC[qtCond] && TWDC[qtCond]>0) // А вдруг этот таймер вовсе не надо запускать?
  {
    cntt2gen = 0;
    // Переход в противоположное состояние, разрешаем/запрещаем генерацию
    qtCond = !qtCond;
    if(qtCond) 
    {
      tk++;
      if(tk<NUM_IMP)
        TIMER_ENABLE_PWM;
      else
        if(tk>NUM_IMP+NUM_EMP) tk=0;
    }
    else
      TIMER_DISABLE_PWM;
  }

  // Время релаксации
  Tcnt++;
  if(Tcnt>WTITER_RAT)
  {
    Tcnt = 0;
    tm[0].t++;
    tm[1].t++;
    tm[2].t++;
    tm[3].t++;
  }

  //
  // Измерения
  //
  for(byte i=0;i<NTS;i++)
  {
    byte sig = digitalRead(TSOP_PIN[i]);  
    if(tm[i].stat==ST_COUNT && sig==0) tm[i].cnt++;

    // Отладочный вывод для TSOP[0]
    //if(i==0) digitalWrite(PIN_DEBUG, sig);

    if(sig!=tm[i].spred)
    {
      tm[i].spred = sig;
      tm[i].t = 0;
      if(sig) // Конец пачки
      {
        if(tm[i].stat==ST_COUNT)
        {
          tm[i].stat = ST_START;
          tm[i].res = tm[i].cnt;
        }
      }
      else  // Начало пачки      
      {
        if(tm[i].stat==ST_START)
        {
          tm[i].stat = ST_COUNT;
          tm[i].cnt = 0;
        }
      }
    }
  }
}

/*
 * 
 */
void iralib::Init(void)
{  
  // Настройка портов
  pinMode(PIN_OUT, OUTPUT);

  pinMode(PIN_DEBUG, OUTPUT);

  for(byte i=0;i<NTS;i++)
  {
    pinMode(TSOP_PIN[i], INPUT);
    digitalWrite(TSOP_PIN[i], HIGH);
  }

  // Настройка прерываний
  cli(); // Stop interrupts

  //----------------------------------------------
  //set timer1 interrupt at WTFREQ
  //----------------------------------------------
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  // Set compare match register for 1hz increments: OCR1A = 1999 = (16*10^6)/(1000*8) - 1
  OCR1A = SYSCLOCK/(WTFREQ*8)-1;

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);  

  //----------------------------------------------
  // set timer2 interrupt
  //----------------------------------------------
  TIMER_CONFIG_KHZ(FTSOP/1000);

  sei(); // Allow interrupts
}

/*
 * 
 */
void iralib::Evaluate(byte result[])
{
  // Разбираемся с инерционностью датчиков
  for(byte i=0;i<NTS;i++)
  {
    if(tm[i].t>LAG_TIME)
    {
      tm[i].t = tm[i].res = 0;
      tm[i].cnt = 0;
    }  
    if(tm[i].t != tm[i].pred_t)
    {
      rb[i].Add(tm[i].res);
      tm[i].pred_t = tm[i].t;
    }

    byte val = rb[i].GetVal();

    result[i] = val;
    short int sp = rb[i].Speed();

    if(abs(sp)>MSPEED) 
    {
      tm[i].res = 0;
      tm[i].t = 0;
      tm[i].cnt = 0;
      result[i] = 0;
    }
  }
}
