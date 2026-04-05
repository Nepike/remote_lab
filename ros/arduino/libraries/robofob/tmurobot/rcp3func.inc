/**
 * Работа с регистрами. Скорость
 * Используется в проектах: mpctl, lpctl, tmu
 * 
 *  \author Robofob
 *  \version 1.25
 *  \date 03.12.2016 / 24.08.2017
 *  \date LP 14.04.2020
 */

#ifndef _RCP3FUNC_INC_
#define _RCP3FUNC_INC_

#include "tmucmd.h"

namespace tmr
{
  byte CSPEED = 0;    /// Крейсерская скорость (pwmspeed)
  byte RSPEED = 0;    /// Скорость разворота (CSPEED*2/3)
};

void Regs2CtlVars(void)
{
  pidlib::maxU = REG_EEP[EEP_MAXU];
  pidlib::minU = -pidlib::maxU; // REG_EEP[EEP_MINU];
  pidlib::minSpeed = REG_EEP[EEP_MINSPEED];  // Скорость трогания
  pidlib::maxSpeed = REG_EEP[EEP_MAXSPEED];
  
  // Производные величины
  pidlib::maxErr = ((float)pidlib::maxSpeed);  // Максимальная ошибка управления

  /***
  byte pwmspeed = REG[REG_PWMSPEED];    // Базовая скорость движения
  tmr::CSPEED = (pwmspeed/5.1);       //(pwmspeed*2/3);                // Крейсерская скорость
  tmr::RSPEED  = (tmr::CSPEED);        //(tmr::CSPEED*2/3);            // Скорость разворота
  ***/

  // Производные величины
  pidlib::maxErr = ((float)pidlib::maxSpeed);  // Максимальная ошибка управления
  tmr::CSPEED = REG_EEP[EEP_PWMSPEED];       // Крейсерская скорость (Базовая скорость движения)
  tmr::RSPEED  = tmr::CSPEED*2/3;            // Скорость разворота
}

#endif
