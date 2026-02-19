/*
 * I2C RC5 Server IRCCtl
 * Transmitter & Reciever
 * Uses 32 bit IR frames for code blocks to carry 16 bit messages. 
 * Has error checking using repetition algorithm.
 * В прошивке осуществляется в том числе кодирование и декодирование сообщений. 
 * Передаются 16 битные сообщения в виде 32 битных пакетов. Сообщение кодируется следующим образом: 
 * сами 16 бит сообщения пишутся в последние 2 байта и копируются в первые 2 байта. Передаются по ИК связи 4 байта. 
 * Если обе половины пакета совпали, то он корректен. Иначе - произошла ошибка. Возвращаются 2 нижних байта, 
 * т.к. они даже в случае ошибки чаще всего сохраняются (ошибка в первую очередь затрагивает верхние 2 байта). 
 * Ответ посылается по I2C основному контроллеру. Контроллеру-мозгу приходят 10 байт. Первый - адрес устройства ИК связи 
 * на I2C линии. Далее следуют 4 пары (сначала верхний, потом нижний байты) байтов соответственно четырёх 
 * ИК приёмников (сторон приёмника). Последний байт содержит биты, которые выставляются в единицу, 
 * если соотв. пакет битый. Самый нижний бит соотв. самому левому пакету в массиве. 
 * Рекомендуемая частота обмена - не более 5 Гц.
 * V 1.21
 * 21.03.2015
 * LP 16.03.2017
 *
 */

// Общий формат принимаемой команды: <cmd> <количество байт> <байты данных>

#include <Wire.h>
#include "i2cwctl.h"
#include "rc5lib.h"

char *Title = "RC5 Recv-Transm Server 1.21";

#define RC5numdigits 32     // количество разрядов в пакете RC5 (не более 32)
// NOTE: определение количества пакетов RC5 для передачи I2C данных (функция I2CData2RC5Data)
// зависит от этого значения, но настраивается вручную; аналогично зависят функции чтения в loop()
// и пересылки по I2C в requestI2CEvent

// Подключение излучателя
// PIN3 is output
IRsend Sender;

// Подключение приемников TSOP
int RECV_PIN_0 =  8;
int RECV_PIN_1 =  9;
int RECV_PIN_2 = 10;
int RECV_PIN_3 = 11;

IRrecv irrecv[RC5Server::NUMRECV];

decode_results results[RC5Server::NUMRECV];

// Буфер приема RC5
unsigned long rc5recievedata[RC5Server::NUMRECV][2]; // 2 элемента использовались в другой версии
  // для хранения длинных сообщений из нескольких частей; в данной нужна лишь первая часть;
  // массив сохранён без изменений, чтобы не менять связанный код

//------------------------------------------------------------

#define I2C_MAX_DATA 20

byte I2Cdata[I2C_MAX_DATA];  // данные I2C
byte I2Cnum = 0;             // количество байт I2C
byte RC5data_delay = 50;     // задержка между данными в пакете RC5

#define RC5_MAX_DATA 20
unsigned long RC5Data[RC5_MAX_DATA];
byte RC5packet_num = 0; // количество пакетов в RC5

// Формирование пакетов RC5
void I2CData2RC5Data(void)
{
  RC5packet_num = I2Cnum/2;
  int n = 0;
  for(int i=0;i<I2Cnum;i+=2)
  {
    unsigned long b1 = ((unsigned long)(I2Cdata[i]) << 8) & 0xFF00;
    unsigned long b0 = I2Cdata[i+1];
    RC5Data[n] = b1 | b0;
    encode_repetition(RC5Data[n]);
    
    n++;
  }
}

// Кодируем последние 2 байта повторением в первых двух
void encode_repetition(unsigned long &data)
{
  data &= 0xFFFFUL; // zero first 2 bytes
  data |= (data << 16); // copy last two bytes two the first
}

// Декодирует блок кода block в сообщение msg
// Используется код повторением первой половины во второй (по 2 байта)
// Возвращает False, если код был с ошибкой
// Сам по себе алгоритм слабый, но для текущего способа обмена сообщениями
// хорошо подходит, т.к. обычно при повреждении сообщения отрезается его первая часть
boolean decode_repetition(unsigned long block, unsigned long &msg)
{
  unsigned int high = (unsigned int) (block >> 16);
  unsigned int low = (unsigned int) (block & 0xFFFFUL);
  
  msg = low;
  
  if (high == low)
    return true;
  else
    return false;
}

// Отправка пакета RC5
void RC5SendPacket(void)
{
  for(int i=0;i<RC5packet_num;i++)
  {
    Sender.sendRC5(RC5Data[i], RC5numdigits);
    // NOTE: delay лучше избегать, однако это требует изменения логики основного цикла
    delay(RC5data_delay);
  }
  // Разрешение приема (Start the receiver)
  enableIRIn();  
}

// Анализ и вывод результатов
// С отладочным выводом над быть очень осторожным: он занимает очень много времени
void OutputResults(void)
{
  for(int i=0;i<RC5Server::NUMRECV;i++)
  {
    // NOTE: mistakes on buffer length here can lead to data corruption (overflow)
    char buf[9];
    sprintf(buf, "%08lx|", rc5recievedata[i][0]);
    Serial.print(buf);
    sprintf(buf, "%08lx", rc5recievedata[i][1]);
    Serial.print(buf);
    Serial.print(' ');
  }
  Serial.println();
}

//------------------------------------------------------------

byte I2CCMD = 0;
byte I2Ccommand_ready = 0;

//----------------------------------------------------------------
// События: прием запросов от мастера
//----------------------------------------------------------------
void receiveI2CEvent(int howMany)
{
  I2Ccommand_ready = 0;

  // Чтение передаваемого пакета
  I2CCMD = I2CDCS::ReadI2CpacketData(&I2Cnum, I2Cdata);

  if(I2CCMD == RC5Server::DCMD_CLEAR_DATA)
  {
    I2Ccommand_ready = 1;
  }
  else
  if(I2CCMD == RC5Server::DCMD_STOP_GENERATE)
  {
    I2Ccommand_ready = 1;
  }
  else
  if(I2CCMD == RC5Server::DCMD_SEND_ONCE || I2CCMD == RC5Server::DCMD_GENERATE)
  {
    // Формирование пакетов RC5
    I2CData2RC5Data();
  }
  else
    return;
  I2Ccommand_ready = 1;
}

void requestI2CEvent()
{
  byte buff[RC5Server::DATALEN];
  int n = 0;  

  // байт, в котором записана информация о битых пакетах
  byte damaged_packages_flag = 0x00;
  
  // Отправляем полученные данные
  // Буфер приема RC5
  for(int i=0;i<RC5Server::NUMRECV;i++)
  {
    byte d0 = (byte)(rc5recievedata[i][0] >> 8);
    byte d1 = (byte)(rc5recievedata[i][0] & 0x00FF);
    buff[n] = d0;
    n++;
    buff[n] = d1;
    n++;

    // check if the package is damaged
    if (rc5recievedata[i][0] >> 31)
      damaged_packages_flag |= (0x01 << i); // NOTE: the leftmost package corresponds to the 
                                            // rightmost bit flag
  }  

  buff[n] = damaged_packages_flag;
  
  Wire.write(buff, RC5Server::DATALEN);
}

//------------------------------------------------------------

void setup()
{
  // join i2c bus with address #RC5I2C::ADDR
  Wire.begin(RC5Server::ADDR);

  // register events
  Wire.onRequest(requestI2CEvent);
  Wire.onReceive(receiveI2CEvent);

  Serial.begin(9600);
  Serial.println(Title);

  irrecv[0].SetPin(0, RECV_PIN_0);
  irrecv[1].SetPin(1, RECV_PIN_1);
  irrecv[2].SetPin(2, RECV_PIN_2);
  irrecv[3].SetPin(3, RECV_PIN_3);

  // Разрешение приема (Start the receiver)
  enableIRIn();  

  memset(rc5recievedata, 0, sizeof(rc5recievedata));

}

//------------------------------------------------------------

byte GenerateRegime = 0;

// Задержка между пакетами RC5 (100). Лучше так, нежели использовать delay
// С учётом периода основного цикла программы (10 мс), частота передачи будет приблизительно равной
// F = 1000 / (TG_DELAY * 10) Hz
#define TG_DELAY 20
int TG = 0;

// Время "релаксации" сигнала. Нужна для имитации инерционности
// Варьируемый параметр. Подбирается экспериментально.
#define TRELAX 50
// Счетчики времен релаксации
int TRCNT[RC5Server::NUMRECV] = {0, 0, 0, 0};

boolean rcvSuccess; // флаг успешного принятия ИК пакета
boolean isDamagedPackage; // флаг битого пакета (сторона приёмника)
boolean anyPackageDamaged; // флаг битого пакета (в целом)

void loop() 
{
  //----------------------------------------------
  // Разбираемся с командой
  //----------------------------------------------
  if(I2Ccommand_ready)
  {
    if(I2CCMD == RC5Server::DCMD_STOP_GENERATE)
    {
      GenerateRegime = 0;
    }
    if(I2CCMD == RC5Server::DCMD_CLEAR_DATA)
    {
      memset(rc5recievedata, 0, sizeof(rc5recievedata));
    }
    if(I2CCMD == RC5Server::DCMD_SEND_ONCE)
    {
      // Отправка пакета RC5
      RC5SendPacket();
    }
    if(I2CCMD == RC5Server::DCMD_GENERATE)
    {
      GenerateRegime = 1;
      TG = TG_DELAY;
    }
    I2Ccommand_ready = 0;
  }
  //----------------------------------------------
  // IR-Recieving
  //----------------------------------------------
  rcvSuccess = false;
  anyPackageDamaged = false;
  for(int i=0;i<RC5Server::NUMRECV;i++)
  {
    isDamagedPackage = false;
    if(irrecv[i].decode(&results[i]))
    {
      TRCNT[i] = 0;  
      // Разбираемся с принятым числом 
      unsigned long data = results[i].value;
      // Serial.println(results[i].rawlen); // print number of read levels
      isDamagedPackage = !decode_repetition(data, rc5recievedata[i][0]); // write results
      anyPackageDamaged |= isDamagedPackage;

      if (isDamagedPackage)
        rc5recievedata[i][0] |= (0x01UL << 31); // mark as damaged
          // NOTE: this variable takes 4 bytes, but the information that is put there takes 
          // only 2 bytes. So, the first 2 bytes are free to be used for extra information.
      
      rc5recievedata[i][1] = data; // full package copy
      /* Старая версия для нескольких частей пакета
      // Накладываем маску для выделения нужных разрядов
      if(data & RC5Server::MASK_PACKET1) // 1-й пакет
        rc5recievedata[i][0] = data;
      else // 2-й пакет
        rc5recievedata[i][1] = data;
      */
      rcvSuccess = true;
      
      irrecv[i].resume(); // Receive the next value
    }
    TRCNT[i]++;
    if(TRCNT[i]>TRELAX) // 'Гасим' сигнал
    {
      TRCNT[i]=TRELAX;
      rc5recievedata[i][0] = rc5recievedata[i][1] = 0;
    }
  }
  //----------------------------------------------
  // IR-Transmission
  //----------------------------------------------
  if(GenerateRegime)
  {
    TG++;
    if(TG>TG_DELAY)
    {
      TG = 0;
      // Отправка пакета RC5
      RC5SendPacket();
      // Лучше не использовать
      // byte RC5packet_delay = 50;   // задержка между пакетами RC5
      // delay(RC5packet_delay);
    }  
  }
  
  if (rcvSuccess)
  {
    if (anyPackageDamaged)
      Serial.print("Damaged package!");
    
    // Анализ и вывод результатов
    OutputResults(); 
  }
    
  delay(10);
}
