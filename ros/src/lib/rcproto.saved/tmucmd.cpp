/**
  \file tmucmd.h
  Протокол - rcX2
  \version 2.30
  \author Robofob

  Коды команд и регистры для архитектуры TMU

  \date 18.01.2014
  \date LP 29.06.2020
*/
#include "tmucmd.h"

TPair RAM_REG_NAMES[] =
{
  "REG_VERSION",       0,
  "REG_ANG_STEP",      1,
  "REG_STOP_SPEED",    2,
  "REG_SPEED",         3,
  "REG_LOC_ENABLE",    4,
  "REG_ACK",           5,
  "REG_USR",           6,

  "REG_D1",            7,
  "REG_D2",            8,

  "REG_STATUS",        9,

  "REG_ENC_LEFT_D0",  10,
  "REG_ENC_LEFT_D1",  11,
  "REG_ENC_RIGHT_D0", 12,
  "REG_ENC_RIGHT_D1", 13,
  "REG_CURR_ANG",     14
};

TPair EEPROM_REG_NAMES[] =
{
  "EEP_ID",            0,
  "EEP_TM_INTERRUPT",  1,
  "EEP_MINU",          2,
  "EEP_MAXU",          3,
  "EEP_MINSPEED",      4,
  "EEP_MAXSPEED",      5,
  "EEP_CHECK_U",       6,

  "EEP_USS_DIST",      7,
  "EEP_BUMP_DIST",     8,

  "EEP_KDB_MOV",       9,
  "EEP_KDB_ROT",      10,

  "EEP_GMT_RWD_LO",   11,
  "EEP_GMT_RWD_HI",   12,

  "EEP_GMT_RECNT_LO", 13,
  "EEP_GMT_RECNT_HI", 14,

  "EEP_GMT_RLW_LO",   15,
  "EEP_GMT_RLW_HI",   16,

  "EEP_PID_KP",       17,
  "EEP_PID_KI",       18,
  "EEP_PID_KD",       19,

  "EEP_SENS_ENABLE",  20,

  "EEP_REFLEX",       21,
  "EEP_PWMSPEED",     22,
  "EEP_TM_STOP",      23
};
