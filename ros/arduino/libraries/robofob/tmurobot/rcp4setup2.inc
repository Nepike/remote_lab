/**
 * Фрагмент инициализации
 *
 *  \author Robofob
 *  \version 1.24
 *  \date 03.12.2016
 *  \date LP 08.04.2020
 */

#ifndef _RCP4SETUP2_INC_
#define _RCP4SETUP2_INC_

  Beep(1);
  robot.Stop();

  lcd.setCursor(0, 1);
  lcd.print("                ");

  lcd.setCursor(0, 0);
  lcd.print("            :");
  lcd.print(ROBOT_ID);


  //robot.reStartMove(0, CSPEED, CSPEED);
  //robot.reStartRotate(90, RSPEED);

#endif
