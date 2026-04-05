
/*
 * Название:			Приёмник ИК-сигналов
 * Автор:				robofob/asanmalyshev(Александр Малышев)
 * Версия:				v2.0
 * Дата создания:		29.03.2019
 * Последнее изменение: 29.03.2019
 *
 */

 /* TODO:
	-	rc5lib_rcv: 
		в функции decode концевик пакета убирается смещением, 
		однако другие поля структуры decode_results, не корректируются.
*/

#include "rc5lib_rcv.h"

const char *Title = "Beacon-reciever v2.0";

//============================================================
//-------------------------- MAIN ----------------------------
//============================================================

#define RCV_NUM	4

#define RCV1 	8
#define RCV2 	9
#define RCV3 	10
#define RCV4 	11

IRrecv irrecv[RCV_NUM];				// Приёмники
decode_results results[RCV_NUM]; 	// Cтруктуры, для хранения результатов дешифровки

#define BAUD_RATE 9600

void setup()
{
	Serial.begin(BAUD_RATE);
	Serial.println("IR Recv demo");
	enableIRIn(); // Start the receiver 
	irrecv[0] = IRrecv(0, RCV1);
	irrecv[1] = IRrecv(1, RCV2);
	irrecv[2] = IRrecv(2, RCV3);
	irrecv[3] = IRrecv(3, RCV4);
}

void loop()  
{	  
	bool flag = false;

	for(unsigned int i=0; i<RCV_NUM; i++){
		 if (irrecv[i].decode(&results[i])){
		 	Serial.print('\t');
		 	Serial.print(i);
		 	Serial.print(": ");
			Serial.print(results[i].value, HEX);

			irrecv[i].resume(); // Receive the next value
			flag=true;
		 }
	}
	if (flag) Serial.println();

}  

