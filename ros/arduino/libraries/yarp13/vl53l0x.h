/*
 * 8 датчиков - лазерных дальномеров VL53L0X 
 * Arduino Mega
 *
 * V 1.3
 *
 * 11.02.2022, 11.05.2023
 * LP 30.07.2023
 * 
 * Для Arduino MEGA 2560 PRO Embed
 * D0  TX  UART
 * D1  RX  UART
 * 
 * D16 TX2 UART
 * D17 RX2 UART
 *
 * D18 TX1 UART
 * D19 RX1 UART
 * 
 * D20 SDA I2C
 * D21 SCL I2C
 */

#include "Adafruit_VL53L0X.h"

namespace vl53
{

const int US_MAX_DIST = 2000; // Максимальная дистанция, мм

// Актуальное количество датчиков
int CLOX_NUM = 0;

// Максимальное количество датчиков
const int MAX_NUM_LOX = 8;

// Счетчики для работы с несчитываемыми данными
#define MAX_WNT 1000   // Ограничение значения счетчика
#define WNT 20 // 20

// Вектор расстояний, см/мм
int Distances[MAX_NUM_LOX];

/*******************
Серверная часть
*********************/
int zcnt[MAX_NUM_LOX];

// Адреса сенсоров
#define LOX_ADDRESS_START 0x30 //0x30  // Возможный начальный адрес (начало поиска)
#define LOX_ADDRESS_END   0x7F //0x7F  // Возможный финальный адрес

// Вектор адресов
byte LOX_ADDR[MAX_NUM_LOX] = {0, 0, 0, 0, 0, 0, 0, 0};

// Порты shutdown
int SHT_LOX_PINS[MAX_NUM_LOX] = {4, 5, 6, 7, 8, 9, 10, 11};

// Датчики vl53l0x
Adafruit_VL53L0X *lox[MAX_NUM_LOX];

// Измерения
VL53L0X_RangingMeasurementData_t measure[MAX_NUM_LOX];

static char sline[60];

void ClientInit(Stream &st)
{
  memset(Distances, 0, sizeof(Distances));
  st.setTimeout(100);
}

void ServerInit()
{
  memset(Distances, 0, sizeof(Distances));
  memset(zcnt, 0, sizeof(zcnt));
  // all init and reset
  for(byte i=0;i<MAX_NUM_LOX;i++)
  {
    lox[i] = new Adafruit_VL53L0X;
    pinMode(SHT_LOX_PINS[i], OUTPUT);
    digitalWrite(SHT_LOX_PINS[i], LOW);
  }
}

int RequestSensors(Stream &stream)
{
  while(stream.available()) stream.read();
  stream.write('?');
  unsigned char buff[MAX_NUM_LOX*sizeof(int)];
  int n = stream.readBytes((char *)Distances, sizeof(Distances));
  for(byte i=0;i<MAX_NUM_LOX;i++)
    Distances[i] = constrain(Distances[i], 0, US_MAX_DIST);
  return n;
}

//
// Последовательное чтение значений дальномеров -> Distances[]
// Возвращает номер считанного сенсора
// sm = true - значения в см. Иначе - в мм.
//
int ReadSensors1(int16_t dist[], bool sm = true)
{
  static int n=0;
  int r = 0;
  bool valid = false;
  lox[n]->rangingTest(&measure[n], false); // pass in 'true' to get debug data printout!
  if(measure[n].RangeStatus != 4)          // if not out of range
  {
    r = measure[n].RangeMilliMeter;
    if(r>US_MAX_DIST) r = US_MAX_DIST;
    zcnt[n] = 0;
    valid = true;
  }
  else // Разбираемся с несчитанными значениями
  {
    if(zcnt[n]<MAX_WNT) zcnt[n]++;
    if(zcnt[n]>WNT)
    {
      r = US_MAX_DIST;
      valid = true;
    }
  }
  if(valid)
  {
    if(sm) r = r/10;
    dist[n] = r;
  }
  n++;
  if(n>=CLOX_NUM) n = 0;
  return n;
}

inline int ReadSensors1(bool sm = true) {  return ReadSensors1(Distances, sm); }

//
// Чтение всех датчиков сразу
//
int ReadSensors(bool sm = true)
{
  for(byte n=0;n<CLOX_NUM;n++)
  {
    int r = 0;
    bool valid = false;
    lox[n]->rangingTest(&measure[n], false); // pass in 'true' to get debug data printout!
    if(measure[n].RangeStatus != 4)          // if not out of range
    {
      r = measure[n].RangeMilliMeter;
      if(r>US_MAX_DIST) r = US_MAX_DIST;
      zcnt[n] = 0;
      valid = true;
    }
    else // Разбираемся с несчитанными значениями
    {
      if(zcnt[n]<MAX_WNT) zcnt[n]++; // Чтобы просто ограничить значение счетчика
      if(zcnt[n]>WNT)
      {
        r = US_MAX_DIST;
        valid = true;
      }
    }
    if(valid)
    {
      if(sm) r = r/10;
      Distances[n] = r/10;
    }
    // Задержка
    #define DEL_TM 5
    unsigned long cms = millis();
    while(millis()-cms<DEL_TM);
  }
  return 0;
}


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
int SetSensorsID(Stream &st, boolean debug)
{
  // All reset
  for(byte i=0;i<MAX_NUM_LOX;i++)
    digitalWrite(SHT_LOX_PINS[i], LOW);
  delay(10);

  // Определяем адреса
  byte caddr = LOX_ADDRESS_START;

  while(CLOX_NUM<MAX_NUM_LOX && caddr<LOX_ADDRESS_END)
  {
    if(debug)
    {
      sprintf(sline, "Try dev %d at addr %x ... ", CLOX_NUM, caddr);
      st.print(sline);
    }

    digitalWrite(SHT_LOX_PINS[CLOX_NUM], HIGH);
    delay(5);
    if(lox[CLOX_NUM]->begin(caddr))
    {
      if(debug) st.print("ok");
      LOX_ADDR[CLOX_NUM] = caddr;
      CLOX_NUM++;
    }
    if(debug) st.println();
    if(CLOX_NUM<MAX_NUM_LOX)
      digitalWrite(SHT_LOX_PINS[CLOX_NUM], LOW);
    caddr++;
  }

  for(byte i=0;i<CLOX_NUM;i++)
    digitalWrite(SHT_LOX_PINS[i], HIGH);
  if(debug)
  {
    st.println("-----------------------------");
    for(byte i=0;i<CLOX_NUM;i++)
    {
      st.print(i);
      st.print(" ");
      st.println(LOX_ADDR[i], HEX);
    }
  }
  return CLOX_NUM;
}

}
