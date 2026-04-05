/*
 * I2C Servers
 * V 1.10
 * 
 * I2CDCS: Общие функции I2C
 * I2CLPW: Протокол системы управления рулевым колесом RWPx1
 * I2CDataServer: Работа с дополнительным сервером данных
 * I2CUSonicServer: Работа с сервером УЗД (Arduino Nano, 6 USS)
 * I2CUSonicServerM8: Работа с сервером УЗД (Arduino Mega, 8 USS)
 * I2CRC5DSRV: Работа с сервером RC5
 * RC5IRAServer: Работа с сервером RC5/IRA
 * I2CColorServer: Работа с бампером
 * 
 * 28.08.2015/05.02.2016/01.12.2022
 * LP 06.02.2023
*/

#ifndef _COMPRO_LIB_
#define _COMPRO_LIB_

// i2c-адреса

#define ADDR_LPW_SERVER         0x04
#define ADDR_VISION_SENSOR      0x09
#define ADDR_DATA_SERVER        0x19
#define ADDR_USONOC_SERVER      0x29

#define ADDR_RC5_SERVER         0x39

#define ADDR_RC5D_RCV_SERVER  0x40 /// i2c адрес приемника RC5D
#define ADDR_RC5D_TRN_SERVER  0x41 /// i2c адрес передатчика RC5D

#define ADDR_COLOR_SERVER       0x49
#define ADDR_SIGNAL_SERVER_1    0x70  /// i2c адрес излучателя
#define ADDR_SIGNAL_SERVER_2    0x80  /// i2c адрес приемника
#define ADDR_SERVO_SERVER       0x83
#define ADDR_USONOC_SERVER_M8   0x93


#ifndef byte
  typedef unsigned char byte;
#endif
/*
 * Область системы управления без ОС (например, рулевой привод)
 */
namespace I2CLPW
{
  const int ADDR = ADDR_LPW_SERVER; /// i2c адрес устройства

  /// Коды команд
  const byte DCMD_SET_ANG   = 0x01; /// Повернуть на угол. Аргумент - первый байт.
                                    /// Рассматривается, как знаковое число - угол (от -128 до +128)
  const byte DCMD_CALIBRATE = 0x02; /// Калибровка
  const byte DCMD_SET_SPEED = 0x03; /// Установить скорость вращения. Аргумент - первый байт.
                                    /// Рассматривается, как беззнаковое число - от 0 до 255
}

/*
 * Область сервера данных
 */
namespace I2CDataServer
{
  const int ADDR = ADDR_DATA_SERVER; /// i2c адрес устройства

  const unsigned int ADC_NUM = 8;    /// Количество АЦП
  const unsigned int PORT_NUM = 14;  /// Количество портов

  const unsigned int DATALEN = (ADC_NUM+PORT_NUM);
};

/*
 * Область сервера УЗД, Arduino Nano, 6 USS
 */
namespace I2CUSonicServer
{
  const int ADDR = ADDR_USONOC_SERVER;  /// i2c адрес устройства

  const unsigned int USONIC_NUM = 6;    /// Количество USonic

  const unsigned int DATALEN = (USONIC_NUM*2); // При условии, что в Arduino sizeof(int)=2

  typedef union
  {
    byte rawdata[DATALEN];
    int dist[USONIC_NUM];
  } USonicData;
};


/*
 * Область сервера УЗД, Arduino Mega, 8 USS
 */
namespace I2CUSonicServerM8
{
  const int ADDR = ADDR_USONOC_SERVER_M8;  /// i2c адрес устройства

  const unsigned int USONIC_NUM = 8;       /// Количество USonic

  const unsigned int DATALEN = (USONIC_NUM*2); // При условии, что в Arduino sizeof(int)=2

  typedef union
  {
    byte rawdata[DATALEN];
    int dist[USONIC_NUM];
  } USonicData;
};

/*
 * Область сервера RC5
 */
namespace RC5DSRV
{
  /*
   Приемник
  */
  const int ADDR_RCV = ADDR_RC5D_RCV_SERVER;  // i2c адрес приемника

  const unsigned int NUMRECV = 4;       // Количество приемников

  const unsigned int DATALEN = (NUMRECV*sizeof(unsigned long)); // Общая длина буфера приема

  // --- Команды ---
  const byte CMD_RCV_SET_RC5    = 0x01; // Установка режима приема пакетов RC5. Аргументы: нет
  const byte CMD_RCV_CLEAR_DATA = 0x02; // Очистка буфера данных RC5. Аргументы: нет

  // --- Интерфейсные функции ---
  // Отправить команду приемнику
  void RcvSendCmd(byte cmd);

  // Получение данных от сервера
  void RcvGetData(unsigned long rcvdata[]);

  /*
   Передатчик
  */
  const int ADDR_TRN = ADDR_RC5D_TRN_SERVER; // i2c адрес передатчика

  // --- Команды ---
  const byte CMD_GEN_RC5  = 0x11;   // Генерация пакетов по протоколу RC5

  /*
   * Управление маяком
   * c[0] - команда (что делать с данными)
   * c[1] - количество передаваемых байт в команде (длина аргументов), 6
   * c[2] - D3 (D3-D0 - отправляемый сигнал)
   * c[3] - D2
   * c[4] - D1
   * c[5] - D0
   * c[6] - количество разрядов
   * c[7] - частота повторения сигнала, Гц (0 - останов генерации)
   */
  void SetBeacon(byte cmd, unsigned long data, byte numdigits, byte freq);
};

/*
 * Область сервера RC5
 */
namespace RC5Server
{
  const int ADDR = ADDR_RC5_SERVER;     /// i2c адрес устройства

  const unsigned int NUMRECV = 4;       /// Количество приемников

  const unsigned int DATALEN = (NUMRECV*2*2); // Общая длина буфера приема

  // --- Команды ---
  const byte DCMD_SEND_ONCE = 0x02;   // Разовая отправка команды по протоколу RC5. Аргументы: <передаваемый пакет I2C>
  const byte DCMD_GENERATE  = 0x03;   // Постоянная генерация пакетов по протоколу RC5. Аргументы: <передаваемый пакет I2C>
                                      // <передаваемый пакет>: <количество байт I2C>  <данные I2C>

  const byte DCMD_STOP_GENERATE = 0x04;   // Останов генерации пакетов. Аргументы: нет
  const byte DCMD_CLEAR_DATA    = 0x05;   // Очистить данные буфера RC5. Аргументы: нет

  // --- Маски ---
  // 12 digits
  // The 1-st packet:  1ddd dddd dddd 
  // Use OR mask to set digits for sending
  // Use AND mask to detect the 1-st digit
  const unsigned int MASK_PACKET1 = 0x0800;

  // The 2-d packet:   0ddd dddd dddd
  // Use AND mask to set digits for sending
  const unsigned int MASK_PACKET2 = 0x07FF;


  inline byte FormDataBuff2(unsigned int d0, unsigned int d1, byte buff[])
  {
    // Устнавливаем 1 для первого пакета и 0 для второго
    d0 = d0 | RC5Server::MASK_PACKET1;
    d1 = d1 & RC5Server::MASK_PACKET2;

    buff[0] = (byte)(d0>>8);
    buff[1] = (byte)(d0 & 0x00FF);

    buff[2] = (byte)(d1>>8);
    buff[3] = (byte)(d1 & 0x00FF);
    return 4;
  }

  // Формирование битового пакета.
  // Пакет формируется, как есть.
  // Разряды номера пакета устанавливаются пользователем
  inline byte FormRawDataBuff(unsigned int d, byte buff[])
  {
    buff[0] = (byte)(d>>8);
    buff[1] = (byte)(d & 0x00FF);
    return 2;
  }

  // Формирование битового пакета.
  // dnum - номер пакета (0 или 1)
  // В старшем разряде пакета устнавливается 1 для первого пакета и 0 для второго
  inline void FormDataBuff1(byte dnum, unsigned int d, byte buff[])
  {
    // Устнавливаем 1 для первого пакета и 0 для второго
    if(dnum==0)
      d = d | RC5Server::MASK_PACKET1;
    else
      d = d & RC5Server::MASK_PACKET2;
    FormRawDataBuff(d, buff);
  }
};

/*
 * Область визуального датчика
 */
namespace I2CVisionSensor
{
  const int ADDR = ADDR_VISION_SENSOR;

  // --- Команды ---
  const byte CMD_ROTATE_H = 0x01;
  const byte CMD_ROTATE_V = 0x02;
  const byte CMD_FOLLOW = 0x03;
  const byte CMD_SCAN = 0x04;
  const byte CMD_ADD_O = 0x05;
  const byte CMD_DEL_O = 0x06;
  const byte CMD_GET = 0x07;
  const byte CMD_US = 0x08;
  const byte CMD_SEARCH = 0x09;  
  const byte CMD_CANCEL = 0x10;  
  const byte CMD_LIGHTER = 0x11;  

  // --- Статус устройства ---
  const byte STATUS_OK = 0x00;  
  const byte STATUS_NOT_LOADED = 0x01;  
  const byte STATUS_NO_CAM = 0x02;  
  const byte STATUS_NO_WPI = 0x03;  
  const byte STATUS_AGR_ERROR = 0x04;  


  const unsigned int DATALEN = 32;

  // --- Данные от сенсора ---
  struct SampleInfo_
  {
    byte ObjectID;
    byte TrackID;
    byte Square;
    byte Size;
  };
  typedef struct SampleInfo_ SampleInfo;

  struct ParsedData_
  {
    byte status;
    byte servoHorisontal;
    byte servoVertical;
    byte USrange;
    SampleInfo sampleData[9];  
  };

  typedef struct ParsedData_ ParsedData;

  ParsedData ParsePacket(byte *packet, int len);
};

namespace I2CColorServer
{
  const int ADDR = ADDR_COLOR_SERVER; /// i2c адрес устройства

  const unsigned int ADC_NUM = 5;    /// Количество АЦП под датчики линии (2), под датчик багажника и 2 передних дальномера
  const unsigned int PORT_NUM = 4;  /// Количество портов под цвета + нажатие бампера

  const unsigned int DATALEN = (ADC_NUM+PORT_NUM+1); //+1 - на регистр статуса
};

namespace I2CSignalServer
{
  const int ADDR =  ADDR_SIGNAL_SERVER_1;  /// i2c адрес излучателя
  const int ADDR2 = ADDR_SIGNAL_SERVER_2;  /// i2c адрес приемника

  const unsigned int DATALEN = 5;
};

 
namespace I2CServoServer
{
  const int ADDR = ADDR_SERVO_SERVER; /// i2c адрес

  const unsigned int DATALEN = 2;
  
  // --- Команды ---
  // Формат пакета: dev angle_low angle_hi
  const byte CMD_SET_MS = 0x01;
  const byte CMD_SET_DR = 0x02;
};


/*
 * Общая серверная область
 */
namespace I2CDCS
{
  void SendCommand1I(byte addr, byte cmd, int data);
  void SendCommand2B(byte addr, byte cmd, byte data1, byte data2);
  void SendCommand(byte addr, byte cmd, byte datanum, byte databuff[]);

  /// Получение данных от сервера
  int ReadData(byte addr, unsigned int size, byte buff[]);

  /// Чтение передаваемого пакета
  /// Возвращает код команды и формирует параметры len и buff
  byte ReadI2CpacketData(byte *len, byte buff[]);

  const unsigned int DATALEN = (I2CDataServer::DATALEN>I2CUSonicServer::DATALEN?I2CDataServer::DATALEN:I2CUSonicServer::DATALEN);
}

#endif

