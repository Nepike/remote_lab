#ifndef _RCPFUNC_INC_
#define _RCPFUNC_INC_

/**
 * Работа с буферами. Определение TSens
 * Функции чтения/записи сенсорной информации, WriteI2CData
 * Используется в проектах: mpctl, lpctl, tmu
 *
 *  \author Robofob
 *  \version 1.25
 *  \date 03.12.2016
 *  \date LP 09.02.2018
 */

/// ----------------------------------------------------------------
/// Высокоуровневые функции
/// ----------------------------------------------------------------

/// Здесь хранятся значения датчиков
#define ADCNUM 8
#define NUMSENS (ADCNUM+4) // АЦП+4 дополнительных

#define BUF_LEN 4 // Длина буфера

TRBuffer<byte, BUF_LEN> Sens[NUMSENS];     /**< Сенсоры. "Сырые" данные */

/// ----------------------------------------------------------------

void ReadSensors(boolean read89 = true)
// Чтение сенсоров. Читать лучше не в цикле, а по одному
{
  static byte n = 0;
  Sens[n].Add((byte)(analogRead(n)>>2));
  n++;
  if(n>=ADCNUM) n = 0;

  if(read89)
  {
    // Пользовательские порты 8 и 9
    Sens[8].Add((byte)digitalRead(8));
    Sens[9].Add((byte)digitalRead(9));
    // Остальные пропускаем
  }
}

void WriteSensors(byte addr)
// Отправка пакета со значениями сенсоров и регистров
// Сначала отсылаются сенсорные данные (NUMSENS байт), а затем - регистры (NREG байт)
{
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(addr);
  pckg.rcWriteBytePtr(pckg.MY_ADDR);
  pckg.rcWriteBytePtr(CMD_ANS_GET_SENS);
  pckg.rcWriteBytePtr(NUMSENS+NREG);
  for(byte i=0;i<NUMSENS;i++)
    pckg.rcWriteBytePtr(Sens[i].GetVal());
  for(byte i=0;i<NREG;i++)
    pckg.rcWriteBytePtr(REG[i]);
  pckg.rcWriteBytePtr(CS_VALUE);
}

void GetAllRegs(byte addr)
{
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(addr);
  pckg.rcWriteBytePtr(pckg.MY_ADDR);
  pckg.rcWriteBytePtr(CMD_ANS_GET_ALL_REG);
  pckg.rcWriteBytePtr(NREG);
  for(byte i=0;i<NREG;i++)
    pckg.rcWriteBytePtr(REG[i]);
  pckg.rcWriteBytePtr(CS_VALUE);
}

/// ----------------------------------------------------------------
//           4  3  2  1  0
//  obstval: R2 L2 C1 R1 L1
#define OBST_USF_LEFT   0x01
#define OBST_USF_RIGHT  0x02
#define OBST_USF_CENTER 0x04
#define OBST_BUMP_LEFT  0x08
#define OBST_BUMP_RIGHT 0x10

/// ----------------------------------------------------------------

/// Функция, прерывающая выполнение движения
int fevent2(void)
{
  int wb = rcWasByte();
  int ob = DetectObstacle();
  if(wb || ob)
  {
    robot.Stop();
    return 1;
  }
  return 0;
};

/// ----------------------------------------------------------------

/// Буфер данных, получаемых от I2C-серверов
/// Полагаем, что I2CDCS::DATALEN - максимальный размер данных I2CDataServer и I2CUSonicServer

unsigned char DSDATA[I2CDCS::DATALEN];

void WriteI2CData(byte addr, unsigned char i2caddr, unsigned int size, unsigned char buff[])
{
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(HDR_BYTE);
  pckg.rcWriteBytePtr(addr);
  pckg.rcWriteBytePtr(pckg.MY_ADDR);
  pckg.rcWriteBytePtr(CMD_ANS_GET_I2C_DATA);
  pckg.rcWriteBytePtr((unsigned char)size+1); // Длина пакета данных (учтем адрес)
  pckg.rcWriteBytePtr(i2caddr); // Сначала посылаем i2c адрес, чтоб было понятно, от какого устройства эти данные
  for(byte i=0;i<size;i++)
    pckg.rcWriteBytePtr(buff[i]);
  pckg.rcWriteBytePtr(CS_VALUE);
}

#endif
