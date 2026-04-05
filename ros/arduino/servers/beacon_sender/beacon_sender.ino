
/*
 * Название:			Излучатель ИК-сигналов
 * Автор:				robofob/asanmalyshev(Александр Малышев)
 * Версия:				v2.0
 * Дата создания:		29.03.2019
 * Последнее изменение: 29.03.2019
 *
 */

#include "rc5lib_sndr.h"

//------------------- Настройки ------------------------------

// Задержка между пакетами RC5
// 114 ms is standard for RC-5
#define REPETITION_DELAY  114

#define IMPULSE_PER_TIME 1 // количество отправляемых сигналов за раз 

const char *Title = "Beacon v2.0";

//--------------- Отправляеямые CMD -------------------------

#define BAUD_RATE 9600

#define RC5numdigits  8	// количество передаваемых разрядов
#define MSG1		0x0AAB //B101010101011
#define MSG2		0x000F
#define MSG3		0x00F1
#define MSG4		0x0F01
#define MSG5		0x0111
#define MSG6		0x0E7C
#define MSG7		0x0C15
#define MSG8		0x0A13
#define MSG9    0x008A

#define MSG 		MSG9

//------------------------------------------------------------

// Отправить пакет 
// Sender 	- объект-отправитель
// cmd 		- отправляемая команда
// repeats 	- количество отправляемых пачек
// ndigits 	- количество разрядов данных
// void sendCmd(IRsend *Sender, byte cmd, byte repeats, byte ndigits);
void sendCmd_long(IRsend *Sender, unsigned long cmd, byte repeats, byte ndigits);

//============================================================
//-------------------------- MAIN ----------------------------
//============================================================

void setup()
{
	Serial.begin(BAUD_RATE);

	Serial.println(Title);
#if defined(__AVR_ATmega2560__) 
	Serial.println("Controller: ATmega2560");
#else 
	Serial.println("Controller: not ATmega2560");
#endif

}
 
IRsend sender(2);

unsigned long msg = MSG;
unsigned int cnt=0;

#define FREQ	10

#define PKGS		5
#define PKG_MAX 	10
void loop()
{
	if (cnt < PKG_MAX) {
		sendCmd_long(&sender, msg, IMPULSE_PER_TIME, RC5numdigits);
		delay(1000/ FREQ);
	}
	Serial.println(cnt);

}

//============================================================

// void sendCmd(IRsend *Sender, byte cmd, byte repeats, byte ndigits)
// {
//   	Serial.print("cmd=");
//   	Serial.println(cmd, BIN);
// 	unsigned long data;
// 	data = cmd;
// 	for(byte i=0; i<repeats; i++)
// 	{
// 		// Отправка пакета RC5
// 		Sender->sendRC5(data, ndigits);
// 		delay(REPETITION_DELAY);  //According to the standard, repetition rate should be 114ms
// 	}
// } 

void sendCmd_long(IRsend *Sender, unsigned long cmd, byte repeats, byte ndigits)
{
  	Serial.print("cmd=");
  	Serial.println(cmd, BIN);
	unsigned long data;
	data = cmd;
	for(byte i=0; i<repeats; i++)
	{
		// Отправка пакета RC5
		Sender->sendRC5(data, ndigits); // 2 статовых бита + ndigits разрядов данных
		delay(REPETITION_DELAY);  //According to the standard, repetition rate should be 114ms
	}
} 
