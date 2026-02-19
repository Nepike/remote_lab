/*
  Протокол Dynamixel, RS485
  Version 1.08
  13.06.2010
  10.03.2013
  LP 31.01.2015

  В основной программе должны быть определены
    int TDRead(char *c);
    int TDWrite(char c);
    void TDSleep(int msec);
*/

#include "dynamixel.h"

BYTE STATUS = 0; //код ошибки (ERROR)

//---------------------------------------------------------------------------
// Задержки
#define WRDTIME 1 // Задержка чтения пакета, мс
#define WWRTIME 2 // Задержка записи пакета, мс

void write_package(BYTE p[], BYTE len)
{
  TDSleep(WWRTIME);
  BYTE i;
  for(i=0;i<len;i++)
    TDWrite(p[i]);
}

//---------------------------------------------------------------------------
// DYNAMIXEL functions
//---------------------------------------------------------------------------
void cmd_set_reg_int_val(BYTE ID, BYTE Register, int Val)
{
  #define LEN_SET_REG_INT 5
  BYTE buff[9];
  BYTE Lo, Hi, CS;

  Lo = low(Val);
  Hi = high(Val);

  // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + ...+ Parameter N )
  CS = (BYTE)~(ID + LEN_SET_REG_INT + CMD_WRITE_DATA + Register + Lo + Hi);

  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = ID;
  buff[3] = LEN_SET_REG_INT;
  buff[4] = CMD_WRITE_DATA;
  buff[5] = Register;
  buff[6] = Lo;
  buff[7] = Hi;
  buff[8] = CS;
  write_package(buff,9);
}

void cmd_set_reg_byte_val(BYTE ID, BYTE Register, BYTE Val)
{
  #define LEN_SET_REG_BYTE 4
  BYTE buff[8];
  BYTE CS;

  // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + ...+ Parameter N )
  CS = (BYTE)(~(ID + LEN_SET_REG_BYTE + CMD_WRITE_DATA + Register + Val));

  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = ID;
  buff[3] = LEN_SET_REG_BYTE;
  buff[4] = CMD_WRITE_DATA;
  buff[5] = Register;
  buff[6] = Val;
  buff[7] = CS;
  write_package(buff,8);
}

UINT Angle2Pos(UINT a)
{
  UINT p;
  p = (UINT)(((float)a-MIN_ANG)/(MAX_ANG-MIN_ANG)*(MAX_POS-MIN_POS)+MIN_POS);
  return p;
}

UINT Pos2Angle(UINT p)
{
  UINT a;
  a = (UINT)(((float)p-MIN_POS)/(MAX_POS-MIN_POS)*(MAX_ANG-MIN_ANG)+MIN_ANG);
  return a;
}

void cmd_set_pos(BYTE ID, int Pos)
{
  cmd_set_reg_int_val(ID, REG_GoalPositionL, Pos);
}

void cmd_set_speed(BYTE ID, int Speed, int dir)
{
  unsigned int DATA;

  if(Speed<0) Speed = 0;
  if(Speed>0x03FF) Speed = 0x03FF;

  // 0x20 <- speed (L), 0x21 <- speed (H)
  DATA = Speed;
  if(dir) DATA |= (1<<10);
  cmd_set_reg_int_val(ID, 0x20, DATA);
}

void cmd_set_angle(BYTE ID, int ang)
// Установить угол (от MIN_ANG до MAX_ANG)
{
  UINT Pos;
  Pos = Angle2Pos(ang);
  cmd_set_pos(ID,Pos);
}

void cmd_write_command_no_args(BYTE ID, BYTE Cmd)
{
  #define LEN_CMD 2
  BYTE buff[6];
  BYTE CS;
  // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + ...+ Parameter N )
  CS = (BYTE)~(ID + LEN_CMD + Cmd);
  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = ID;
  buff[3] = LEN_CMD;
  buff[4] = Cmd;
  buff[5] = CS;
  write_package(buff,6);
}

void cmd_set_id(BYTE OLD_ID, BYTE NEW_ID)
{
  #define LEN_CMD_SET_ID 4
  BYTE buff[8];
  BYTE CS;
  // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + ...+ Parameter N )
  CS = (BYTE)~(OLD_ID + LEN_CMD_SET_ID + 0x03 + 0x03 + NEW_ID);
  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = OLD_ID;
  buff[3] = LEN_CMD_SET_ID;
  buff[4] = 0x03;
  buff[5] = 0x03;
  buff[6] = NEW_ID;
  buff[7] = CS;
  write_package(buff,8);
}

//---------------------------------------------------------------------------

void cmd_read_data(BYTE ID, BYTE Addr, BYTE DataLen)
{
  #define LEN_READ_DATA 4
  BYTE buff[8];
  BYTE CS;

  // Check Sum = ~ ( ID + Length + Instruction + Parameter1 + ...+ Parameter N )
  CS = (BYTE)(~(ID + LEN_READ_DATA + CMD_READ_DATA + Addr + DataLen));

  buff[0] = 0xff;
  buff[1] = 0xff;
  buff[2] = ID;
  buff[3] = LEN_SET_REG_BYTE;
  buff[4] = CMD_READ_DATA;
  buff[5] = Addr;
  buff[6] = DataLen;
  buff[7] = CS;
  write_package(buff,8);
}

//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------

int read_package(BYTE package[])
// Чтение пакета. Возвращает длину пакета или:
//  0 - истек таймаут
// -1 - ошибка заголовка пакета
{
  BYTE ID, LEN;
  int i;

  TDSleep(WRDTIME);
  // read 0xff, 0xff
  if(!TRead(&package[0])) return 0;
  if(!TRead(&package[1])) return -2;

  if(package[0]!=0xFF || package[1]!=0xFF) return -3;

  if(!TRead(&package[2])) return -4;
  ID = package[2];

  if(!TRead(&package[3])) return -5;
  LEN = package[3];

  for(i=0;i<LEN;i++)
    if(!TRead(&package[i+4])) return -6;
  /*
  STATUS (package[4]) содержит код ошибки (ERROR)
  */
  STATUS = package[4];

  return (int)LEN+4;
}

//---------------------------------------------------------------------------
// DYNREAD
//---------------------------------------------------------------------------

/* Макросы с номерами байт в пакете */
#define BYTE_HEAD     0
#define BYTE_HEAD_2   1
#define BYTE_ID       2
#define BYTE_LENGTH   3
#define BYTE_ERROR    4

void dynread_init(dynread_state_t *state, unsigned char * output)
{
  state->position = 0;
  state->id = 0;
  state->length = 0;
  state->error = 0;
  state->message = output;
}

dynread_error_t dynread_read(int (*getch_cb)(unsigned char *),
                             dynread_state_t *state)
{
  unsigned char input;
  state->state = DYNREAD_IN_PROGRESS;

  while (getch_cb(&input))
  {
    switch (state->position)
    {
      case BYTE_HEAD:
      case BYTE_HEAD_2:
        if (0xFF != input) state->state = DYNREAD_BROKEN_PACKAGE;
        break;

      case BYTE_ID:
        state->id = input;
        break;

      case BYTE_LENGTH:
        state->length = input;
        if (state->length > DYNREAD_MESSAGE_MAX_LENGTH)
        {
          state->state = DYNREAD_LENGTH_ERROR;
        }
        break;

      case BYTE_ERROR:
        state->error = input;
        if (input) state->state = DYNREAD_HARDWARE_ERROR;
        break;

      /* Остальные байты в последовательности - параметры
       * и контрольная сумма */
      default:
        /* Проверка контрольной суммы */
        if (state->position >= state->length + 3)
        {
          int i;
          unsigned char checksum = 0;
          for (i = 2; i < state->length + 3; ++i)
          {
            checksum += state->message[i];
          }

          checksum = ~checksum;
          if (checksum != input)
          {
            state->state = DYNREAD_CHECKSUM_ERROR;
          }
          else
          {
            state->position = 0;
            return state->state = DYNREAD_DONE;
          }
        }
        break;
      }

      if (state->state != DYNREAD_IN_PROGRESS)
      {
        state->position = 0;
      }
      else
      {
        state->message[state->position] = input;
        ++state->position;
      }
  }

  return state->state;
}
