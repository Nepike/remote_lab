//
// V 3.03
// 31.12.2014
// LP 02.01.2015
//

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "emplib.h"

//----------------------------------------------------------
//
//----------------------------------------------------------
#define BYTE unsigned char

//----------------------------------------------------------

int UseBrakingGen   = 1;   // Флаг использования режима торможения выходных генераторов
float EmotionsCoeff = 0.5; // Коэффициент эмоциональной связи

//----------------------------------------------------------
// Все сигналы - от 0 до 1
//----------------------------------------------------------
// Нейрон
//----------------------------------------------------------

float delta_t = 0.05;  // Шаг времени
float Noise   = 0.10;  // Шум
float P0      = 0.10;  // Порог

// Коэффициенты для вычисления входного воздействия
float Iemax     = 0.50;  // ? не используется
float q_exiting = 1.00;  // Коэффициент для входов возбуждения
float d_braking = 1.00;  // Коэффициент для входов торможения

// Выходные параметры
float Omega       = 1.00;  // Коэффициент зависимости частоты генерации
float OmegaMax    = 1.00;  // Максимальная частота генерации импульсов
float OmegaMin    = 0.00;  // Минимальная частота генерации импульсов

float Alpha       = 0.10;  // Коэффициент затухания (чем больше, тем быстрее затухание)
float EmThreshold = 0.40;  // Порог эмоций для генерации символа (коэффициент болтливости)

//----------------------------------------------------------
// Правило
//----------------------------------------------------------

void Normalize(float &r)
{
  if(r < 0) r = 0;
  if(r > 1) r = 1;
}

float AddNoise(float r)
{
  float d;
  d = r*Noise;
  r = (float)rand()*2*d/RAND_MAX+(r-d);
  Normalize(r);
  return r;
}

void TElement::Init(ElType ntip, float tP, TElement *lNeed, BYTE nisout)
// lNeed - потребность, связанная с правилом
{
  isout = nisout;
  tip = ntip;
  outf = tmpoutf = 0;

  // Нейрон
  P = tP;
  numExcitingSrc = 0;
  numBrakingSrc = 0;
  UCT = 0;

  // Правило
  numDis = 0;
  Need = lNeed;
  E = 0;
  memset(DisConSrc,0,sizeof(DisConSrc));
}

float TElement::CountOutput(void)
// Расчет выходной активности элемента
// (выходной активности нейрона или ct правила)
{
  float df;
  if(tip!=N_RULE) // Выходная активность нейрона
  {
    // Если тип - генератор, то будем просто выдавать U
    if(tip==N_GEN)
    {
      tmpoutf = UCT;
      float r = AddNoise(tmpoutf);
      return r;
    }
    df = UCT - P;
    if(df>0)
      tmpoutf = UCT;// sic Omega*df+OmegaMin;
    else
      tmpoutf = OmegaMin;
    if(tmpoutf>OmegaMax) tmpoutf=OmegaMax;
    return tmpoutf;
  }
  else // Правило
  {
    tmpoutf = UCT;
    return tmpoutf;
  }
}

float TElement::GetOutput(void)
// Выходная активность элемента (выходная активность нейрона или ct правила)
{
  float r;
  r = AddNoise(outf);
  return r;
}

void TElement::Commit(void)
{ outf = tmpoutf; }

float TElement::GetNeed(void)
// Получить значение потребности, связанной с элементом
{
  float r;
  if(!Need)
    r = 1;
  else
    r = Need->GetOutput();
  return r;
}

void TElement::RuleAddCon(TElement *lnk, BYTE dnum, BYTE isneg)
// numDis - номер дизъюнкта
{
  BYTE k;
  if(dnum>=MAX_DIS) error("E02: dis_num>=MAX_DIS");
  if(dnum + 1 > numDis)
    numDis=(BYTE)(dnum+1);
  // Количество коньюнктов не храним, поэтому надо вычислять
  k=0;
  while(DisConSrc[dnum][k]!=NULL) k++;
  if(k>=MAX_CON) error("E03: k>=MAX_CON");

  DisConSrc[dnum][k] = lnk;
  negCon[dnum][k] = isneg;
}

void TElement::SetUCT(float fct)
// Установить значение потенциала U или ct
{
  UCT = fct;
  if(tip!=N_RULE)
    Normalize(UCT);
}

void TElement::NeuronSetExcitingLink(TElement *lnk)
{
  if(numExcitingSrc>=MAX_EXCITING_NIMP)
    error("numExcitingSrc is too large");
  ExcitingSrc[numExcitingSrc] = lnk;
  numExcitingSrc++;
}

void TElement::NeuronSetBrakingLink(TElement *lnk)
{
  if(numBrakingSrc>=MAX_BRAKING_NIMP)
    error("numBrakingSrc is too large");
  BrakingSrc[numBrakingSrc] = lnk;
  numBrakingSrc++;
}

float TElement::NeuronCountInp(void)
// Вычисление суммарных входных воздействий
{
  BYTE i;
  float S = 0;
  float d;
  for(i=0;i<numExcitingSrc;i++)
  {
    d = ExcitingSrc[i]->GetOutput();
    //S += d*Iemax*(1 - UCT*q_exiting); // sic
    S += d*q_exiting;
  }
  for(i=0;i<numBrakingSrc;i++)
  {
    d = BrakingSrc[i]->GetOutput();
    S -= d*d_braking;
  }

  Normalize(S);

  return S;
}

void TElement::NeuronCountU(void)
// Вычисление потенциала нейрона U
{
  float X;
  if(tip!=N_NEURON) return;

  // Входная импульсация X
  X = NeuronCountInp();

  /* Реакция на вход X
     Теоретические варианты:
       1. UCT = UCT+10*X*(1-X);
       2. if(UCT<X) UCT = UCT+X*(1-X);
     Рабочие варианты:
       1. if(X>Alpha) UCT = X;
       2. if(X>UCT*Alpha) UCT = X;
       3. if(X>UCT) UCT = X;
       4. UCT = X;
  */

  if(X>UCT) UCT = X;

  /*
    Затухание
    Надо затухать более или менее плавно
    Варианты:
      1. UCT = UCT - Alpha*delta_t*UCT*exp(-delta_t);
      2. UCT = UCT - UCT*UCT;
      3. UCT = UCT - UCT*Alpha;
      4. UCT = UCT - Alpha;
  */
  UCT = UCT - UCT*Alpha;

  Normalize(UCT);
}

float TElement::RuleCountCt(void)
// Вычисление коэффициента уверенности заключения
{
  BYTE i, j;
  float S, c, Cmin;

  if(tip!=N_RULE) return 0;
  S = 0;
  for(i=0;i<numDis;i++) // Цикл по дизьюнктам
  {
    // Вычисление минимального коньюнкта Cmin
    Cmin = 1;
    for(j=0;DisConSrc[i][j]!=NULL;j++) // Цикл по коньюнктам
    {
      c = DisConSrc[i][j]->GetOutput();
      if(negCon[i][j]) c = 1 - c; // Отрицание
      if(Cmin>c) Cmin = c;
    }
    S = Cmin + S - Cmin*S;
  }

  // Если тип - генератор, то будем просто выдавать ct
  if(numDis==0)
    S = UCT;
  UCT = S;
  tmpoutf = S;
  E = 0;
  return S;
}

void TElement::CountU(void)
// Вычисление потенциала нейрона или ct правила
{
  if(tip==N_NEURON)   // Вычисление потенциала нейрона
    NeuronCountU();
  if(tip==N_RULE)     // Вычисление коэффициента уверенности заключения
    RuleCountCt();
}

//----------------------------------------------------------

// Нейроны - потребности (генераторы)
TElement NEED_FOOD(11), NEED_SAVE(12), NEED_COMFORT(13);

// Нейроны - сенсоры (генераторы)
TElement SEN_FOOD(21), SEN_WALL(22), SEN_DANGER(23);

// Правила
TElement R_EAT(31), R_GOTO(32), R_WALK(33), R_RUN_AWAY(34);

// Нейроны - вентили
TElement V_FOOD(41), V_WALL(42), V_DANGER(43);

// Выходные нейроны
TElement OUT_EAT(51), OUT_GOTO(52), OUT_WALK(53), OUT_RUN_AWAY(54);

// Генераторы эмоций
TElement GEM_EAT(61), GEM_GOTO(62), GEM_WALK(63), GEM_RUN_AWAY(64);

// Нейроны - внешние символы (генераторы)
TElement SYMB_V_FOOD(71), SYMB_V_WALL(72), SYMB_V_DANGER(73);

int N_ELEMENTS = 0;
TElement *N[MAX_N_ELEMENTS];

float GlobalT = 0;

//----------------------------------------------------------
//
//----------------------------------------------------------

// Параметры генераторов
// Потребности
float dneed0        = 0.01;
float need_food0    = 0.75;
float need_save0    = 0.50;
float need_comfort0 = 0.65;

// Сенсоры
float dsens0        = 0.01;
float sens_food0    = 0.75;
float sens_wall0    = 0.25;
float sens_danger0  = 0.50;

float dneed = dneed0;
float
  need_food = need_food0,
  need_save = need_save0,
  need_comfort = need_comfort0;

// Сенсоры
float dsens = dsens0;
float
  sens_food = sens_food0,
  sens_wall = sens_wall0,
  sens_danger = sens_danger0;

// Возвращает итоговое значение эмоции
//   action - номер действия 
//   minem  - номер правила с минимальной эмоцией
float CountStep(BYTE SetOutf, BYTE &action, BYTE &minem)
{
  BYTE i;
  int MaxRN;
  float SE;

  GlobalT+=delta_t;

  for(i=0;i<N_ELEMENTS;i++)
  {
/*
if(i==12)
  printf("12");
if(i==23)
  printf("23");
*/
    // Вычисление потенциала нейрона или ct правила
    N[i]->CountU();

    // Выходная активность элемента (активность нейрона или ct правила)
    N[i]->CountOutput();
  }
  //------------------------------------------------------
  // Эмоции
  // Вычисление максимального Ct для выходных правил
  //------------------------------------------------------
  MaxRN = -1;
  // Ищем первое правило
  for(i=0;i<N_ELEMENTS;i++)
    if(N[i]->tip==N_RULE)
    {
      MaxRN=i;
      break;
    }
  minem = (BYTE)MaxRN;
  if(MaxRN>=0) // есть правила
  {
    for(i=0;i<N_ELEMENTS;i++)
      if(N[i]->tip==N_RULE)
        if(N[MaxRN]->UCT < N[i]->UCT) MaxRN=i;
    // Вычисление эмоций
    for(i=0;i<N_ELEMENTS;i++)
      if(N[i]->tip==N_RULE)
      {
        float t;
        if(i!=MaxRN) t = 0; else t = 1;
        N[i]->E = N[i]->GetNeed()*(t - N[i]->UCT);
        if(SetOutf)
          N[i]->tmpoutf = t;
        // Определяемся с минимальной эмоцией
        if(N[i]->E < N[minem]->E) minem = i;
      }
    SE = 0;
    for(i=0;i<N_ELEMENTS;i++)
      SE+=N[i]->E;

    // Генераторы эмоций
    GEM_EAT.SetUCT(R_EAT.E>0?0:fabs(R_EAT.E)*EmotionsCoeff);
    GEM_GOTO.SetUCT(R_GOTO.E>0?0:fabs(R_GOTO.E)*EmotionsCoeff);
    GEM_RUN_AWAY.SetUCT(R_RUN_AWAY.E>0?0:fabs(R_RUN_AWAY.E)*EmotionsCoeff);
    GEM_WALK.SetUCT(R_WALK.E>0?0:fabs(R_WALK.E)*EmotionsCoeff);
  }
  //------------------------------------------------------
  for(i=0;i<N_ELEMENTS;i++)
    N[i]->Commit();

  //------------------------------------------------------
  // Выдаем управляющее воздействие
  // Определяем максимальный выход
  action = 0;
  float maxval = 0;
  for(BYTE i=0;i<N_ELEMENTS;i++)
    if(N[i]->isout)
    {
      float y = N[i]->outf;
      if(y>=maxval) //sic
      {
        maxval = y;
        action = N[i]->id;
      }
    }

  return SE;
}

void ResetParamVals(void)
{
  // Потребности
  NEED_FOOD.SetUCT(need_food);
  NEED_SAVE.SetUCT(need_save);
  NEED_COMFORT.SetUCT(need_comfort);

  // Сенсоры
  SEN_FOOD.SetUCT(sens_food);
  SEN_WALL.SetUCT(sens_wall);
  SEN_DANGER.SetUCT(sens_danger);
}

void SetDefaultParams(void)
{
  dneed = dneed0;
  need_food = need_food0,
  need_save = need_save0,
  need_comfort = need_comfort0;

  dsens = dsens0;
  sens_food = sens_food0,
  sens_wall = sens_wall0,
  sens_danger = sens_danger0;
}

void InitSystem(void)
{
  GlobalT = 0;

  N_ELEMENTS = 24;

  N[0] = &NEED_FOOD;
  N[1] = &NEED_SAVE;
  N[2] = &NEED_COMFORT;

  N[3] = &SEN_FOOD;
  N[4] = &SEN_WALL;
  N[5] = &SEN_DANGER;

  N[6] = &R_EAT;
  N[7] = &R_GOTO;
  N[8] = &R_WALK;
  N[9] = &R_RUN_AWAY;

  N[10] = &V_FOOD;
  N[11] = &V_WALL;
  N[12] = &V_DANGER;

  N[13] = &OUT_EAT;
  N[14] = &OUT_GOTO;
  N[15] = &OUT_WALK;
  N[16] = &OUT_RUN_AWAY;

  N[17] = &GEM_EAT;
  N[18] = &GEM_GOTO;
  N[19] = &GEM_WALK;
  N[20] = &GEM_RUN_AWAY;

  N[21] = &SYMB_V_FOOD;
  N[22] = &SYMB_V_WALL;
  N[23] = &SYMB_V_DANGER;

  //--------------------------------------------------------
  // Инициализация элементов
  //--------------------------------------------------------
  // Потребности
  NEED_FOOD.Init(N_GEN, P0, NULL);
  NEED_SAVE.Init(N_GEN, P0, NULL);
  NEED_COMFORT.Init(N_GEN, P0, NULL);

  // Сенсоры
  SEN_FOOD.Init(N_GEN, P0, NULL);
  SEN_WALL.Init(N_GEN, P0, NULL);
  SEN_DANGER.Init(N_GEN, P0, NULL);

  // Вентили
  V_FOOD.Init(N_NEURON, P0, NULL);
  V_WALL.Init(N_NEURON, P0, NULL);
  V_DANGER.Init(N_NEURON, P0, NULL);

  // Правила
  R_EAT.Init(N_RULE, 0, &NEED_FOOD);
  R_GOTO.Init(N_RULE, 0, &NEED_FOOD);
  R_WALK.Init(N_RULE, 0,  &NEED_COMFORT);
  R_RUN_AWAY.Init(N_RULE, 0, &NEED_SAVE);

  // Генераторы эмоций
  GEM_EAT.Init(N_GEN, P0, NULL);
  GEM_GOTO.Init(N_GEN, P0, NULL);
  GEM_WALK.Init(N_GEN, P0, NULL);
  GEM_RUN_AWAY.Init(N_GEN, P0, NULL);

  // Выходные нейроны
  OUT_EAT.Init(N_NEURON, P0, NULL, 1);
  OUT_GOTO.Init(N_NEURON, P0, NULL, 1);
  OUT_WALK.Init(N_NEURON, P0, NULL, 1);
  OUT_RUN_AWAY.Init(N_NEURON, P0, NULL, 1);

  // Нейроны - внешние символы (генераторы)
  SYMB_V_FOOD.Init(N_NEURON, P0, NULL);
  SYMB_V_WALL.Init(N_NEURON, P0, NULL);
  SYMB_V_DANGER.Init(N_NEURON, P0, NULL);

  //--------------------------------------------------------
  // Связи (правила)
  //--------------------------------------------------------
  R_EAT.RuleAddCon(&NEED_FOOD, 0, IS_POS);
  R_EAT.RuleAddCon(&V_FOOD, 0, IS_POS);

  R_GOTO.RuleAddCon(&NEED_FOOD, 0, IS_POS);
  R_GOTO.RuleAddCon(&V_WALL, 0, IS_POS);
  R_GOTO.RuleAddCon(&V_FOOD, 0, IS_NEG);

  R_WALK.RuleAddCon(&V_WALL, 0, IS_NEG);
  R_WALK.RuleAddCon(&NEED_COMFORT, 0, IS_POS); // sic

  R_RUN_AWAY.RuleAddCon(&NEED_SAVE, 0, IS_POS);
  R_RUN_AWAY.RuleAddCon(&V_DANGER, 0, IS_POS);

  R_RUN_AWAY.RuleAddCon(&NEED_COMFORT, 1, IS_POS);
  R_RUN_AWAY.RuleAddCon(&V_WALL, 1, IS_POS);

  //--------------------------------------------------------
  // Связи (вентили)
  //--------------------------------------------------------
  V_FOOD.NeuronSetExcitingLink(&SEN_FOOD);
  V_WALL.NeuronSetExcitingLink(&SEN_WALL);
  V_DANGER.NeuronSetExcitingLink(&SEN_DANGER);

  //--------------------------------------------------------
  // Возбуждение генераторов
  //--------------------------------------------------------
  OUT_EAT.NeuronSetExcitingLink(&R_EAT);
  OUT_GOTO.NeuronSetExcitingLink(&R_GOTO);
  OUT_WALK.NeuronSetExcitingLink(&R_WALK);
  OUT_RUN_AWAY.NeuronSetExcitingLink(&R_RUN_AWAY);

  //--------------------------------------------------------
  // Обратные связи эмоций с вентилями
  //--------------------------------------------------------
  V_FOOD.NeuronSetExcitingLink(&GEM_EAT);
  V_WALL.NeuronSetExcitingLink(&GEM_GOTO);
  V_WALL.NeuronSetExcitingLink(&GEM_RUN_AWAY);
  V_WALL.NeuronSetExcitingLink(&GEM_WALK);
  V_DANGER.NeuronSetExcitingLink(&GEM_RUN_AWAY);

  //--------------------------------------------------------
  // Торможение генераторов
  //--------------------------------------------------------
  if(UseBrakingGen)
  {
    OUT_EAT.NeuronSetBrakingLink(&OUT_GOTO);
    OUT_EAT.NeuronSetBrakingLink(&OUT_RUN_AWAY);
    OUT_EAT.NeuronSetBrakingLink(&OUT_WALK);

    OUT_GOTO.NeuronSetBrakingLink(&OUT_EAT);
    OUT_GOTO.NeuronSetBrakingLink(&OUT_RUN_AWAY);
    OUT_GOTO.NeuronSetBrakingLink(&OUT_WALK);

    OUT_WALK.NeuronSetBrakingLink(&OUT_EAT);
    OUT_WALK.NeuronSetBrakingLink(&OUT_RUN_AWAY);
    OUT_WALK.NeuronSetBrakingLink(&OUT_GOTO);

    OUT_RUN_AWAY.NeuronSetBrakingLink(&OUT_EAT);
    OUT_RUN_AWAY.NeuronSetBrakingLink(&OUT_GOTO);
    OUT_RUN_AWAY.NeuronSetBrakingLink(&OUT_WALK);
  }

  //--------------------------------------------------------
  // Нейроны - внешние символы (генераторы)
  //--------------------------------------------------------
  V_FOOD.NeuronSetExcitingLink(&SYMB_V_FOOD);
  V_WALL.NeuronSetExcitingLink(&SYMB_V_WALL);
  V_DANGER.NeuronSetExcitingLink(&SYMB_V_DANGER);

  //--------------------------------------------------------

  SetDefaultParams();
  ResetParamVals();
}
