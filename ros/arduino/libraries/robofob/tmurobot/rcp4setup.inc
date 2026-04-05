/**
 * Setup. Настройка портов, LCD, rcX2
 * Используется в проектах: mpctl, lpctl, tmu
 *
 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016
 *  \date LP 08.04.2020
 */

#ifndef _RCP4SETUP_INC_
#define _RCP4SETUP_INC_

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

#ifdef DEBUG_OUTPUT
  stdrobot::debug_output = 1;
#else
  stdrobot::debug_output = 0;
#endif

  EEPROM.get(0, REG_EEP);

  Regs2CtlVars();

  //------------------------------------------------------------------------

  // Протокол rcX2
  if(ROBOT_ID==0 || ROBOT_ID==255) ROBOT_ID = 1;
  TPckgInit(&pckg, ROBOT_ID, rcError, rcReadByte, rcWriteByte, rcWasByte, rcTimeoutEvent, rcResetTimeoutCnt);

  //
  // ПИД-регулятор робота
  //
  // Пропорциональное звено, %, (0..100)
  float Kp = REG_EEP[EEP_PID_KP]/100.0;

  // Интегрирующее звено, %, (0..100)
  float Ki = REG_EEP[EEP_PID_KI]/100.0;

  // Дифференцирующее звено, %, (0..100)
  float Kd = REG_EEP[EEP_PID_KD]/100.0;

  robot.InitPID(Kp, Ki, Kd, -pidlib::maxSpeed*5, pidlib::maxSpeed*5, pidlib::minU, (float)pidlib::maxU, pidlib::maxErr);

  // Период прерываний таймера, мкс
  // Например, 50'000 микросекунд (или 0.05 сек - или 20Hz)
  // REG_EEP[EEP_TM_INTERRUPT] задает период таймера в мс.
  
  unsigned long tns = 1000L * REG_EEP[EEP_TM_INTERRUPT];

  /**
   * Геометрия робота
   *
   * RECnt - Количество импульсов энкодеров на 1 оборот колеса
             (т.к. мы регистрируем изменения сигнала, а не сам сигнал, то будем писать в 2 раза больше)
   * RWD   - Диаметр колеса, мм.
   * RLW   - Расстояние между колесами, мм.
   */
  int RWD = REG_EEP[EEP_GMT_RWD_LO] + (REG_EEP[EEP_GMT_RWD_HI]<<8);
  int RECnt = REG_EEP[EEP_GMT_RECNT_LO] + (REG_EEP[EEP_GMT_RECNT_HI]<<8);
  int RLW = REG_EEP[EEP_GMT_RLW_LO] + (REG_EEP[EEP_GMT_RLW_HI]<<8);

  robot.Init(tns, RECnt, RWD, RLW, PIN_encoderLeft, PIN_encoderRight);

  // Сервомашинка локатора
  myservo.attach(PIN_SERVO_LOCATOR);  // attaches the servo on pin 9 to the servo object
  SetServoAng(0);
  delay(500);

  int mv_speed = pidlib::maxSpeed*REG_EEP[EEP_KDB_MOV]/100.0;
  int rot_speed = pidlib::maxSpeed*REG_EEP[EEP_KDB_ROT]/100.0;

#endif
