//
// V 3.03
// 31.12.2014
// LP 18.01.2015
//

#ifndef _EMPLIB_H_
#define _EMPLIB_H_

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

void error(const char *fmt, ...);

#define BYTE unsigned char

//----------------------------------------------------------
// Тип элемента: нейрон-генератор, "обычный" нейрон, правило
enum ElType {N_GEN, N_NEURON, N_RULE};

//----------------------------------------------------------

extern int UseBrakingGen;
extern float EmotionsCoeff; // Коэффициент эмоциональной связи

//----------------------------------------------------------
// Все сигналы - от 0 до 1
//----------------------------------------------------------
// Нейрон
//----------------------------------------------------------

// Коэффициенты для вычисления входного воздействия
extern float Iemax;     // ? не используется
extern float q_exiting; // Коэффициент для входов возбуждения
extern float d_braking; // Коэффициент для входов торможения

extern float delta_t; // Шаг времени
extern float Noise;   // Шум
extern float P0;      // Порог

// Выходные параметры
extern float Omega;    // Коэффициент зависимости частоты генерации
extern float OmegaMax; // Максимальная частота генерации импульсов
extern float OmegaMin; // Минимальная частота генерации импульсов

extern float Alpha;    // Коэффициент затухания (чем больше, тем быстрее затухание)
extern float EmThreshold; // Порог эмоций для генерации символа (коэффициент болтливости)

// Максимальное количество входов для нейрона
#define MAX_EXCITING_NIMP 5  // Максимальное количество источников возбуждения
#define MAX_BRAKING_NIMP  4  // Максимальное количество источников торможения

//----------------------------------------------------------
// Элемент (правило, нейрон, генератор)
//----------------------------------------------------------
#define MAX_CON 4 // Максимальное количество коньюнктов в правиле
#define MAX_DIS 2 // Максимальное количество дизьюнктов в правиле

#define IS_NEG 1
#define IS_POS 0

struct TElement
{
  //----------------------------------------------
  // Общая часть
  //----------------------------------------------
  BYTE id;       // Идентификатор элемента
  BYTE isout;    // Признак того, что это - выходной элемент

  ElType tip;    // Тип элемента

  float outf;    // Выходная импульсация нейрона или активность правила
  float tmpoutf; // Рабочая (до commit) выходная импульсация (активность)

  float UCT;     // Потенциал нейрона или коэффициент уверенности заключения

  //----------------------------------------------
  // Нейрон
  //----------------------------------------------
  float P;     // Порог нейрона
  BYTE numExcitingSrc;   // Количество входов возбуждения
  BYTE numBrakingSrc;    // Количество входов
  TElement *ExcitingSrc[MAX_EXCITING_NIMP]; // Нейроны-источники входа возбуждения
  TElement *BrakingSrc[MAX_BRAKING_NIMP];   // Нейроны-источники входа торможения

  //----------------------------------------------
  // Правило
  //----------------------------------------------
  TElement *Need;  // Потребность, связанная с правилом
  BYTE numDis;     // Количество дизьюнктов

  TElement *DisConSrc[MAX_DIS][MAX_CON]; // Правила-дизьюнкты-коньюнкты
  BYTE negCon[MAX_DIS][MAX_CON];         // Флаги отрицания (инверсия) коньюнктов

  float E;       // Эмоция

  //----------------------------------------------

  TElement(BYTE nid) { id = nid; }

  // lNeed - потребность, связанная с правилом
  void Init(ElType ntip, float tP, TElement *lNeed, BYTE nisout = 0);

  // Расчет выходной активности элемента
  // (выходной активности нейрона или ct правила)
  float CountOutput(void);

  // Выходная активность элемента (выходная активность нейрона или ct правила)
  float GetOutput(void);

  void Commit(void);

  // Получить значение потребности, связанной с элементом
  float GetNeed(void);

  // Добавление правила в дизъюнкт
  // numDis - номер дизъюнкта
  void RuleAddCon(TElement *lnk, BYTE dnum, BYTE isneg);

  // Установить значение потенциала U или ct
  void SetUCT(float fct);

  void NeuronSetExcitingLink(TElement *lnk);

  void NeuronSetBrakingLink(TElement *lnk);

  // Вычисление суммарных входных воздействий
  float NeuronCountInp(void);

  // Вычисление потенциала нейрона U
  void NeuronCountU(void);

  // Вычисление коэффициента уверенности заключения
  float RuleCountCt(void);

  // Вычисление потенциала нейрона или ct правила
  void CountU(void);
};
//----------------------------------------------------------
// Все сигналы - от 0 до 1
//----------------------------------------------------------

float AddNoise(float r);

#define MAX_N_ELEMENTS 32
extern int N_ELEMENTS;
extern TElement *N[MAX_N_ELEMENTS];

extern float GlobalT;

//----------------------------------------------------------
//
//----------------------------------------------------------
// Параметры генераторов
// Потребности
extern float dneed0;
extern float need_food0;
extern float need_save0;
extern float need_comfort0;

// Сенсоры
extern float dsens0;
extern float sens_food0;
extern float sens_wall0;
extern float sens_danger0;

//----------------------------------------------------------

extern float dneed;
extern float
  need_food,
  need_save,
  need_comfort;

// Сенсоры
extern float dsens;
extern float
  sens_food,
  sens_wall,
  sens_danger;

// Нейроны - потребности (генераторы)
extern TElement NEED_FOOD, NEED_SAVE, NEED_COMFORT;

// Правила
extern TElement R_EAT, R_GOTO, R_RUN_AWAY, R_WALK;

// Нейроны - сенсоры (генераторы)
extern TElement SEN_FOOD, SEN_WALL, SEN_DANGER;

// Нейроны - вентили
extern TElement V_FOOD, V_WALL, V_DANGER;

// Выходные нейроны
extern TElement OUT_EAT, OUT_GOTO, OUT_RUN_AWAY, OUT_WALK;

// Генераторы эмоций
extern TElement GEM_EAT, GEM_GOTO, GEM_RUN_AWAY, GEM_WALK;

// Нейроны - внешние символы (генераторы)
extern TElement SYMB_V_FOOD, SYMB_V_WALL, SYMB_V_DANGER;

void Normalize(float &r);

// Возвращает итоговое значение эмоции
//   action - номер действия 
//   minem  - номер правила с минимальной эмоцией
float CountStep(BYTE SetOutf, BYTE &action, BYTE &minem);

void ResetParamVals(void);
void SetDefaultParams(void);
void InitSystem(void);

#endif
