#ifndef _RCP4SETUP_INC_
#define _RCP4SETUP_INC_

/**
 * Setup. Настройка портов, LCD, rcX2
 * Используется в проектах: mpctl, lpctl, tmu
 *
 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016
 *  \date LP 24.08.2017
 */

  // Настройка com-порта на скорость BAUD_RATE
  PSERIAL.begin(BAUD_RATE);

  if(stdrobot::debug_output)
    PSERIAL.println(Title);

  //------------------------------------------------------------------------
  // Инициализация lcd
  // Внимание! Инициализация идет очень долго - больше секунды.

#ifdef USE_ADA_FRUIT
  #define WHITE  0x7
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
#else
  //lcd.init();
  lcd.begin();
  lcd.backlight();
  lcd.home();
#endif

  // Печатаем первую строку
  // Устанавливаем курсор в колонку 0, строку 0. Нумерация начинается с нуля.
  lcd.setCursor(0, 0);
  lcd.print(Title);
  // Устанавливаем курсор в колонку 0, строку 1
  lcd.setCursor(0, 1);
  lcd.print("Test ...        ");

  // sic Надо ли это? Wire.begin();

#ifdef DEBUG_OUTPUT
  stdrobot::debug_output = 1;
#else
  stdrobot::debug_output = 0;
#endif

  //------------------------------------------------------------------------

  // Разбираемся с идентификатором робота
#ifdef USE_EEPROM_ID
  // Хранение ROBOT_ID в EEPROM
  REG[REG_ID] = EEPROM.read(0);
  if(REG[REG_ID]==0 || REG[REG_ID]==255) 
  // "Чистая" память. Видимо, только что проинициализированная
  // Прописываем туда значение ROBOT_ID
  {
    REG[REG_ID] = ROBOT_ID;
    EEPROM.write(0, ROBOT_ID);
  }
#else
  // EEPROM не используется
  // Определяем каждый раз при запуске
  REG[REG_ID] = ROBOT_ID;
#endif

  ROBOT_ID = REG[REG_ID];
  // Протокол rcX2
  TPckgInit(&pckg, ROBOT_ID, rcError, rcReadByte, rcWriteByte, rcWasByte, rcTimeoutEvent, rcResetTimeoutCnt);

#endif
