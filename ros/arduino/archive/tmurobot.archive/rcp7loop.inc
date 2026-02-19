#ifndef _RCP7LOOP_INC_
#define _RCP7LOOP_INC_

/**
 * Инвариант тела основного цикла
 * Используется в проектах: mpctl, lpctl, tmu
 * 
 *  \author Robofob
 *  \version 1.26
 *  \date 03.12.2016
 *  \date LP 19.02.2018
 */

  static int LEFT_SPEED = robot.GoalSpeedLeft;
  static int RIGHT_SPEED = robot.GoalSpeedRight;
  
  if(obst && (LAST_MOVECMD != CMD_BACK) && (LAST_MOVECMD != CMD_BACK2)
    && !( (LAST_MOVECMD == CMD_SET_SPEED) && (LEFT_SPEED < 0) && (RIGHT_SPEED < 0) ) )
  {
    LAST_MOVECMD = CMD_STOP;
    robot.mvstatus=TRobot::MVC_READY;
    robot.Stop();
  }

  lcd.setCursor(0, 1);
  lcd.print(obst);
  lcd.print(" ");
  
  unsigned char addr, from = 0;
  unsigned char CMD;
  unsigned char i2caddr, i2ccmd, b1, b2;
  int angle;
  byte databuff[10];
  byte datalen;
  unsigned char n, val;
  unsigned int uang;

  if(rcReadPackage(&pckg)==DATA_READY)
  {
    addr = rcGetAddr(&pckg);
    if(addr != REG[REG_ID] && addr != 255) return;
    from = rcGetFrom(&pckg);
    CMD = rcGetCmd(&pckg);
    if(IS_MOVE_CMD(CMD)) //sic
    {
      // Прямое задание скорости
      robot.set_speed_regime = (CMD == CMD_SET_SPEED);
      if(IS_REENTERABLE_MOVING(LAST_MOVECMD) && (CMD != CMD_SET_SPEED))
      {
        robot.Stop();
        robot.mvstatus=TRobot::MVC_READY;
      }
      LAST_MOVECMD = CMD;
    }

    REG[REG_D1] = robot.Cnt2L(stdrobot::GlobalEncoderLeftCnt.val);
    REG[REG_D2] = robot.Cnt2L(stdrobot::GlobalEncoderRightCnt.val);

    REG[REG_ENC_LEFT_D0] = stdrobot::GlobalEncoderLeftCnt.d[0];
    REG[REG_ENC_LEFT_D1] = stdrobot::GlobalEncoderLeftCnt.d[1];

    REG[REG_ENC_RIGHT_D0] = stdrobot::GlobalEncoderRightCnt.d[0];
    REG[REG_ENC_RIGHT_D1] = stdrobot::GlobalEncoderRightCnt.d[1];

    switch(CMD)
    {
      case CMD_PING:
        break;
      case CMD_STOP:
        robot.Stop();
        break;
      case CMD_FWD:
        if(obst) break;
        goFwd(tmr::CSPEED);
        break;
      case CMD_FWD2:
        if(obst) break;
        rcGet1B(&pckg, &val);
        robot.reStartMove(10*val, tmr::CSPEED, tmr::CSPEED);
        break;
      case CMD_BACK:
        goBack(tmr::CSPEED);
        break;
      case CMD_BACK2:
        rcGet1B(&pckg, &val);
        robot.reStartMove(10*val, -tmr::CSPEED, -tmr::CSPEED);
        break;
      case CMD_LEFT:
        if((obst & OBST_BUMP_RIGHT)) break;
        goLeft(tmr::CSPEED);
        break;
      case CMD_RIGHT:
        if((obst & OBST_BUMP_LEFT)) break;
        goRight(tmr::CSPEED);
        break;
      case CMD_FAST_LEFT:
        goFastLeft(tmr::CSPEED);
        break;
      case CMD_FAST_LEFT2:
        rcGet1B(&pckg, &val);
        angle = val;
        robot.reStartRotate(angle, tmr::RSPEED);
        break;
      case CMD_FAST_RIGHT:
        goFastRight(tmr::CSPEED);
        break;
      case CMD_FAST_RIGHT2:
        rcGet1B(&pckg, &val);
        angle = -val;
        robot.reStartRotate(angle, tmr::RSPEED);
        break;
      case CMD_SET_SPEED:
        rcGet2B(&pckg, &b1, &b2); 
        robot.GoalSpeedLeft = LEFT_SPEED = ((int)pidlib::maxSpeed*(signed char)b1/100);
        robot.GoalSpeedRight = RIGHT_SPEED = ((int)pidlib::maxSpeed*(signed char)b2/100);
        break;
      case CMD_BEEP:
        break;
      case CMD_BEEP_ON:
        digitalWrite(PIN_BEEP, HIGH);
        break;
      case CMD_BEEP_OFF:
        digitalWrite(PIN_BEEP, LOW);
        break;
      case CMD_DEBUG:
        break;
      case CMD_GET_SENS:
        // Выдача всех сенсоров и значений регистров
        WriteSensors(from);
        break;
      case CMD_GET_USR_DATA:
        WriteUsrData(from);
        break;
      case CMD_SET_REG:
        rcGet2B(&pckg, &n, &val); 
        REG[n] = val;
        if(n==REG_ID) // Т.к. REG[REG_ID] (MY_ADDR) хранится в EEPROM
        {
          EEPROM.write(0, val);
          pckg.MY_ADDR = val;
        }
        if(n==REG_D1) stdrobot::GlobalEncoderLeftCnt.val = 0;
        if(n==REG_D2) stdrobot::GlobalEncoderRightCnt.val = 0;
        Regs2CtlVars();
        break;
      case CMD_GET_REG:
        rcGet1B(&pckg, &n);
        val = REG[n];
        rcSend1B(&pckg, 0, CMD_ANS_GET_REG, val);
        break;
      case CMD_GET_ALL_REG:
        GetAllRegs(from);
        break;

      //
      //*** Работа с сервомашинкой
      // Установить сервомашинку в позицию ang (угол в градусах от 0 до 180)
      case CMD_SET_SERVO_POS:
        rcGet1B(&pckg, &val);
        uang = val;
        SetServoAng(uang);
        break;
      //
      //*** Работа с I2C-устройствами (серверами)
      //
      // Команда I2C-устройству. Формат: <i2c-адрес устройства> <команда> <количество аргументов> <аргумент1> <аргумент2> ...
      case CMD_I2C:
        rcGetI2CDataBuff(&pckg, &i2caddr, &i2ccmd, &datalen, databuff);
        I2CDCS::SendCommand(i2caddr, i2ccmd, datalen, databuff);
        break;
      // Запрос: Получить массив данных от i2c-устройства. При этом надо заранее знать размер запрашиваемых данных
      case CMD_GET_I2C_DATA:
        rcGet2B(&pckg, &i2caddr, &b1);
        I2CDCS::ReadData(i2caddr, b1, DSDATA);
        WriteI2CData(from, i2caddr, b1, DSDATA);
        break;
      default:
        rcError(ERR_ILLEGAL_CMD);
    }
    if((!IS_NOT_ACK_CMD(CMD) && REG[REG_ACK]) || CMD==CMD_PING)
      rcSendACK(&pckg, 0);
  }

  // Шаг движения
  robot.reMoveStep();

  // Локатор
  if(REG[REG_LOC_ENABLE])
    ServoProc();

  // Регистр статуса
  REG[REG_STATUS] = (robot.mvstatus << 4) | obst;

#endif
