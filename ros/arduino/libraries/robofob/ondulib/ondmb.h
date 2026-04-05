/*

  V 1.4

  08.07.2023, 10.12.2023, 18.01.2024
  LP 27.06.2024

*/

#ifndef _MODB_H_
#define _MODB_H_

//----------------------------------------------------------
//
//----------------------------------------------------------
namespace ondmb
{
//----------------------------------------------------------
// Структура внешних сенсорных данных
//----------------------------------------------------------
const int8_t STAT_NUM = 3; // Статус
const int8_t DIST_NUM = 8; // Количество дальномеров
const int8_t ADC_NUM  = 8; // Количество АЦП
const int8_t DS_NUM   = 4; // Количество двоичных датчиков
const int8_t ENC_NUM  = 1; // Количество энкодеров

// Общее количество элементов (24 слова)
const int8_t EXT_SENS_DATA_LEN = STAT_NUM + DIST_NUM + ADC_NUM + DS_NUM + ENC_NUM;

// 24 слова
struct TSensorsData
{
  int16_t dstat[STAT_NUM]; 
  int16_t ddist[DIST_NUM]; 
  int16_t dadc[ADC_NUM];
  int16_t ddigital[DS_NUM]; 
  int16_t denc[ENC_NUM];
};


// Полезное объединение
union TUSensData
{
  TSensorsData SData;
  int16_t AData[ondmb::EXT_SENS_DATA_LEN];
};

//----------------------------------------------------------
// Структура данных
//----------------------------------------------------------

// Регистры сенсоров
const int8_t SENS_DATA_OFFS =  0;
const int8_t INT_SENS_DATA_LEN =  24; // Количество регистров данных контроллеров
const int8_t SENS_DATA_LEN  = INT_SENS_DATA_LEN + EXT_SENS_DATA_LEN; // 24+24=48 слов

// Регистры команд
const int8_t CMD_DATA_OFFS  = SENS_DATA_LEN;
const int8_t CMD_DATA_LEN   = 14;

// Итоговый размер регистрового файла, 58+4=62 слова
const int8_t DATA_LEN = SENS_DATA_LEN + CMD_DATA_LEN;

int16_t au16data[DATA_LEN];

//----------------------------------------------------------
// Коды команд
//----------------------------------------------------------
const int8_t cmd_set      =  3;
const int8_t cmd_get      =  4;
const int8_t cmd_resetall =  5;
const int8_t cmd_getstat  =  6;
const int8_t cmd_reset    =  7;
const int8_t cmd_dctl     =  8;
const int8_t cmd_info     =  9;
const int8_t cmd_getsens  = 10;
const int8_t cmd_vset     = 11;
const int8_t cmd_vdctl    = 12;

const int8_t cmd_a_enable = 13; // Разрешить работу драйверов
const int8_t cmd_a_disable= 14; // Запретить работу драйверов

const int8_t cmd_uset     = 15;

const int8_t cmd_set_encoder = 16; // Установка значений энкодеров. Аргументы: номер энкодера (0,1), устанавливоемое значение

//----------------------------------------------------------

char sline[80];

// Получение данных от сервера сенсоров
int ReadExternalData(Stream &DataStr, Stream &LogStr, TUSensData &ExtSens)
{
  // Запрос
  DataStr.write('?');
  // Чтение ответа
  int n = DataStr.readBytes((uint8_t*)&ExtSens.SData, sizeof(ExtSens.SData));
  if(n==sizeof(ExtSens.SData))
  {
    // Перегоняем в au16data
    for(byte i=0;i<EXT_SENS_DATA_LEN;i++)
      au16data[SENS_DATA_OFFS+INT_SENS_DATA_LEN+i] = ExtSens.AData[i];
    return n;
  }
  else
  {
    while(DataStr.available()) DataStr.read();
  }
  return 0;
}

void PrintData(Stream &S, const char *title, uint8_t pos, uint8_t len)
{
  S.print(title);
  for(int i=pos;i<pos+len;i++)
  {      
    sprintf(sline, "%u ", au16data[i]);
    S.print(sline);
  }
  S.println();
}

void FormCmd3(int16_t cmd, int16_t arg1, int16_t arg2, int16_t arg3)
{
  au16data[CMD_DATA_OFFS+0] = cmd;
  au16data[CMD_DATA_OFFS+1] = arg1;
  au16data[CMD_DATA_OFFS+2] = arg2;
  au16data[CMD_DATA_OFFS+3] = arg3;
}

int8_t GetCmd3(int16_t &arg1, int16_t &arg2, int16_t &arg3)
{
  int8_t cmd = au16data[CMD_DATA_OFFS+0];
  arg1 = au16data[CMD_DATA_OFFS+1];
  arg2 = au16data[CMD_DATA_OFFS+2];
  arg3 = au16data[CMD_DATA_OFFS+3];
  return cmd;
}

int8_t GetCmd6(int16_t &arg1, int16_t &arg2, int16_t &arg3, int16_t &arg4, int16_t &arg5, int16_t &arg6)
{
  int8_t cmd = au16data[CMD_DATA_OFFS+0];
  arg1 = au16data[CMD_DATA_OFFS+1];
  arg2 = au16data[CMD_DATA_OFFS+2];
  arg3 = au16data[CMD_DATA_OFFS+3];
  arg4 = au16data[CMD_DATA_OFFS+4];
  arg5 = au16data[CMD_DATA_OFFS+5];
  arg6 = au16data[CMD_DATA_OFFS+6];
  return cmd;
}

void PrintSensors(char *prefix, Stream &S, TSensorsData &SData)
{
  S.print(prefix);
  for(byte i=0;i<STAT_NUM;i++)
  {
    sprintf(sline, "%3d ", SData.dstat[i]);
    S.print(sline);
  }
  S.print(" | ");
  for(byte i=0;i<DIST_NUM;i++)
  {
    sprintf(sline, "%3d ", SData.ddist[i]);
    S.print(sline);
  }
  S.print(" | ");
  for(byte i=0;i<ADC_NUM;i++)
  {
    sprintf(sline, "%4d ", SData.dadc[i]);
    S.print(sline);
  }
  S.print(" | ");
  for(byte i=0;i<DS_NUM;i++)
  {
    sprintf(sline, "%3d ", SData.ddigital[i]);
    S.print(sline);
  }
  S.print(" | ");
  for(byte i=0;i<ENC_NUM;i++)
  {
    sprintf(sline, "%3d ", SData.denc[i]);
    S.print(sline);
  }
  S.println();
}

};

//----------------------------------------------------------
//
//----------------------------------------------------------
namespace tm1637
{
// Create an array that turns all segments ON
const uint8_t allON[] = {0xff, 0xff, 0xff, 0xff};

// Create an array that turns all segments OFF
const uint8_t allOFF[] = {0x00, 0x00, 0x00, 0x00};
};

#endif
