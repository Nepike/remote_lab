#ifndef _RCP2FUNC_INC_
#define _RCP2FUNC_INC_

/**
 * Функции реализации протокола rxX2
 * mpctl, lpctl, tmu
 *
 *  \author Robofob
 *  \version 1.6.1
 *  \date 03.12.2016
 *  \date LP 18.11.2017
 */

TPckg pckg;

unsigned char rcReadByte(void) { return PSERIAL.read(); }
void rcWriteByte(unsigned char c) { PSERIAL.write(c); }
unsigned char rcWasByte(void) { return (PSERIAL.available()>0); }
void rcError(unsigned char  n) 
{
  lcd.setCursor(0, 1);
  lcd.print("Error ");
  lcd.print(n);
}

// Проверка таймаута. Возвращает 0, если истекло время ожидания очередного символа пакета
uchar rcTimeoutEvent(void) { return 0; }

// Сброс счетчика таймаута
void rcResetTimeoutCnt(void) { return; }

#endif
