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

#ifndef _P_DYNAMIXEL_H_
#define _P_DYNAMIXEL_H_

extern int TDRead(char *c);
extern int TDWrite(char c);
extern void TDSleep(int msec);

//----------------------------------------------------------
// Диапазон углов и позиций
//----------------------------------------------------------
#define MIN_ANG       0
#define MAX_ANG     300

#define MIN_POS  0x0000
#define MAX_POS  0x03ff

//----------------------------------------------------------
// Диапазон скоростей
//----------------------------------------------------------
#define MIN_SPEED 0x0000
#define MAX_SPEED 0x03ff

//----------------------------------------------------------
// Диапазон моментов
//----------------------------------------------------------
#define MIN_TORQUE 0x0000
#define MAX_TORQUE 0x03ff

//----------------------------------------------------------
// Команды
//----------------------------------------------------------
#define CMD_PING        0x01   // нет параметров
#define CMD_READ_DATA   0x02   // 2 параметра
#define CMD_WRITE_DATA  0x03   // 2 и более параметров
#define CMD_RESET       0x06   // Сброс. Установить "заводские параметры"
#define CMD_SYNC_WRITE  0x83   // This command is used to control 
                               // several RX-64s simultaneously at a time.
                               // 4 or more parameters

//----------------------------------------------------------
// Регистры
//----------------------------------------------------------
// EEPROM
//----------------------------------------------------------
#define REG_MAX_TORQUE_L       0x0E // Max Torque(L) read/write  0xFF
#define REG_MAX_TORQUE_H       0x0F // Max Torque(H) read/write  0x03

#define REG_StatusReturnLevel  0x10 // Status Return Level (RW), dafault value = 0x02

//----------------------------------------------------------
// RAM
//----------------------------------------------------------
#define REG_GoalPositionL  0x1E // Lowest byte of Goal Position (RW)
#define REG_GoalPositionH  0x1F // Highest byte of Goal Position (RW)

#define REG_TorqEnable     0x18 // Torque On/Off (RW)
#define REG_LED            0x19 // LED On/Off (RW)

#define REG_MovSpeedL      0x20 // Moving Speed(L) Lowest byte of Moving Speed (RW)
#define REG_MovSpeedH      0x21 // Moving Speed(H) Highest byte of Moving Speed (RW)

#define REG_TorqLimL       0x22 // Torque Limit(L) Lowest byte of Torque Limit (RW)
#define REG_TorqLimH       0x23 // Torque Limit(H) Highest byte of Torque Limit (RW)

// Set angle ctrl regime (reset operation): 0x08, 0x09 <- 0x03FF
// Rotate regime: 0x08, 0x09 <- 0
#define ADDR_CNT_CLW_A_LIM_L    0x08   // Lowest byte of counterclockwise Angle Limit(L). Init val = 0xFF
#define ADDR_CNT_CLW_A_LIM_H    0x09   // Hightest byte of counterclockwise Angle Limit(H). Init val = 0x03

#define ADDR_PRESENT_POSITION_L 0x24 // Present Position (L)
#define ADDR_PRESENT_POSITION_H 0x25 // Present Position (H)

//----------------------------------------------------------
//
//----------------------------------------------------------
#define PACK_STAT_NO_RETURN      0x00 // No return against all instructions
#define PACK_STAT_ONLY_READ_DATA 0x01 // Return only for the READ_DATA command
#define PACK_STAT_ALL_INSTR      0x02 // Return for all Instructions Note

//----------------------------------------------------------
// Разряды статусного байта
//----------------------------------------------------------
/*
  Bit 7 0  - 
  Bit 6 Instruction Error 
     When undefined Instruction is transmitted or the Action 
     command is delivered without the reg_write command 
  Bit 5 Overload Error 
     When the current load cannot be controlled with theset 
     maximum torque 
  Bit 4 Checksum Error 
     When the Checksum of the transmitted Instruction Packet is invalid 
  Bit 3 Range Erro  When the command is given beyond the range of usage
  Bit 2 Overheating Erro 
     When the internal temperature is out of the range of 
     operating temperature set in the Control Table 
  Bit 1 Angle Limit Erro 
     When Goal Position is written with the value that is not 
     between CW Angle Limit and CCW Angle Limit 
  Bit 0 Input Voltage Error 
     When the applied voltage is out of the range of operating 
     voltage set in the Control Table 
*/
#define STAT_INSTRUCTION_ERROR   (1<<6) //0b01000000
#define STAT_OVERLOADED_ERROR    (1<<5) //0b00100000
#define STAT_CHECKSUM_ERROR      (1<<4) //0b00010000
#define STAT_RANGE_ERROR         (1<<3) //0b00001000
#define STAT_OVERHEATING_ERROR   (1<<2) //0b00000100
#define STAT_ANGLE_LIMIT_ERROR   (1<<1) //0b00000010
#define STAT_INPUT_VOLTAGE_ERROR (1<<0) //0b00000001

//----------------------------------------------------------
//
//----------------------------------------------------------

#define high(N) (BYTE)(N>>8)
#define low(N)  (BYTE)(N & 0xFF)

typedef unsigned char BYTE;
typedef unsigned int UINT;

extern void rs232write(char c);
extern int TRead(BYTE *n);

extern BYTE STATUS; //код ошибки (ERROR)

void write_package(BYTE p[], BYTE len);

//---------------------------------------------------------------------------
// DYNAMIXEL functions
//---------------------------------------------------------------------------
void cmd_set_reg_int_val(BYTE ID, BYTE Register, int Val);

void cmd_set_reg_byte_val(BYTE ID, BYTE Register, BYTE Val);

UINT Angle2Pos(UINT a);
UINT Pos2Angle(UINT pos);

void cmd_set_pos(BYTE ID, int Pos);
void cmd_set_speed(BYTE ID, int Speed, int dir);

// Установить угол (от MIN_ANG до MAX_ANG)
void cmd_set_angle(BYTE ID, int ang);

void cmd_write_command_no_args(BYTE ID, BYTE Cmd);

void cmd_set_id(BYTE OLD_ID, BYTE NEW_ID);

inline void cmd_PING(BYTE ID)
{ cmd_write_command_no_args(ID, CMD_PING); }

inline void cmd_set_stat_return_level(BYTE ID, BYTE Level)
{ cmd_set_reg_byte_val(ID, REG_StatusReturnLevel, Level); }

inline void cmd_torque_enable(BYTE ID, BYTE Enable)
{ cmd_set_reg_byte_val(ID, REG_TorqEnable, Enable); }

void cmd_read_data(BYTE ID, BYTE Addr, BYTE DataLen);

// Чтение пакета. Возвращает длину пакета или:
//  0 - истек таймаут
// -1 - ошибка заголовка пакета
int read_package(BYTE package[]);


//---------------------------------------------------------------------------
// DYNREAD
//---------------------------------------------------------------------------
/*******************************************************************************
 * dynread
 * Дата: 2013-10-22
 *******************************************************************************
 *  Пример использования:
 *
 *   #define DYNREAD_MESSAGE_MAX_LENGTH 127
 *   #include "dynamixel.h"
 *
 *   dynread_state_t state;
 *   dynread_init(&state);
 *
 *   for (;;)
 *   {
 *       dynread_read(getchar_func, &state);
 *       if (state.state == DYNREAD_DONE)
 *       {
 *           printf("ID: %d, length: %d", state.id, state.length);
 *       }
 *   }
 *
 ******************************************************************************/


/* Максимально допустимая длина сообщения - может быть переопределена */
#ifndef DYNREAD_MESSAGE_MAX_LENGTH
#  define DYNREAD_MESSAGE_MAX_LENGTH (255+5)
#endif

/* Возвращаемое значение */
typedef enum
{
    DYNREAD_DONE           = 0, /* Сообщение прочитано до конца успешно */
    DYNREAD_IN_PROGRESS    = 1, /* Чтение сообщения в процессе */
    DYNREAD_BROKEN_PACKAGE = 2, /* Нарушен формат пакета */
    DYNREAD_CHECKSUM_ERROR = 3, /* Ошибка контрольной суммы */
    DYNREAD_HARDWARE_ERROR = 4, /* Отказ оборудования */
    DYNREAD_LENGTH_ERROR   = 5  /* Длина пакета превышает максимум */
}
dynread_error_t;

/* Состояние чтения пакета */
typedef struct
{
    int position;
    unsigned char * message; //[DYNREAD_MESSAGE_MAX_LENGTH];
    unsigned char id, length, error;
    dynread_error_t state;
}
dynread_state_t;

/**
 * @brief Инициализация переменной состояния, необходимая перед ее
 *        использованием
 *
 * @param state
 *          Указатель на переменную состояния
 */
void dynread_init(dynread_state_t *state, unsigned char *output);

/**
 * @brief Неблокирующая функция чтения пакета в формате Dynamixel с сохранением
 *        состояния
 *
 * @param getch_cb
 *          Указатель на неблокирующую функцию, считывающую символ с
 *          устройства ввода-вывода. В случае успеха возвращает 0, иначе - код
 *          ошибки.
 *
 * @param state
 *          Состояние чтения пакета - пользователь функции должен сохранять
 *          эту переменную между итерациями цикла.
 *
 * @return dynread_error_t
 *          Результат чтения пакета.
 */
dynread_error_t dynread_read(int (*getch_cb)(unsigned char *),
                             dynread_state_t *state);


#endif
