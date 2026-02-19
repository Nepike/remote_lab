#ifndef _RCP3FUNC_INC_
#define _RCP3FUNC_INC_

/**
 * Работа с регистрами. Скорость
 * Используется в проектах: mpctl, lpctl, tmu
 * 
 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016
 *  \date LP 24.08.2017
 */

namespace tmr
{
  byte CSPEED = 0;    /// Крейсерская скорость (pwmspeed*2/4)
  byte RSPEED = 0;    /// Скорость разворота (CSPEED*2/3)
};

void Regs2CtlVars(void)
{
  pidlib::maxU = REG[REG_MAXU];
  pidlib::minU = -pidlib::maxU; //REG[REG_MINU];
  pidlib::minSpeed = REG[REG_MINSPEED];  // Скорость трогания
  pidlib::maxSpeed = REG[REG_MAXSPEED];
  
  byte pwmspeed = REG[REG_PWMSPEED];    // Базовая скорость движения

  // Производные величины
  pidlib::maxErr = ((float)pidlib::maxSpeed);  // Максимальная ошибка управления
  tmr::CSPEED = (pwmspeed*2/3);                // Крейсерская скорость
  tmr::RSPEED  = (tmr::CSPEED*2/3);            // Скорость разворота
}

#endif
