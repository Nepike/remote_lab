
/*
 * Название:			Приёмо-передатчик ИК-сигналов
 * Автор:				robofob/asanmalyshev(Александр Малышев)
 * Версия:				v1.1
 * Дата создания:		2019.07.29
 * Последнее изменение: 2020.02.05
 *
 */

#include "rc5LibReciever.h" 
#include "rc5LibSender.h"	
#include "i2cwctl.h"
#include "iralib.h"
#include "Wire.h"

//------------------- SETTINGS ------------------------

const char *Title = "Beacon-reciever with IRA v1.1";

#define DEBUG	
#define EMITTER

#define BAUD_RATE 9600 			

#define TSOP_KEEP_IN_MEMORY_TIME 250 // in mc
#define REPEATS_NUM   1     // количество повторных отправок одного и того же пакета 

#define TSOP_NUM	4	

#define PIN_TSOP_1 	8
#define PIN_TSOP_2 	9
#define PIN_TSOP_3 	10
#define PIN_TSOP_4 	11

// I2C settings
#define ADDR_RCV	0x80
#define ADDR_EMIT 	0x70

#ifdef EMITTER
  #define ADDR ADDR_EMIT
#else
  #define ADDR ADDR_RCV
#endif

#define I2C_BUF_LEN 5		
#define I2C_MAX_DATA 20

#define RC5numdigits  8			// количество передаваемых разрядов	

// ===== CMDS =====
#define CMD_EMIT_STOP 	    0
#define CMD_EMIT_START_RC5  1
#define CMD_EMIT_START_IRA 	2
#define CMD_SET_IRA_MODE   	3
#define CMD_SET_RC5_MODE   	4

// ===== MODS =====
#define NOT_START   	      0xFF
//------------------------------------------------------------

#define REPETITION_DELAY  114	// delay between RC5 packages (114 ms is standard for RC-5)

// Отправить пакет 
// Sender 	- объект-отправитель
// value 		- отправляемое значение  
// repeats 	- количество отправляемых пачек
// ndigits 	- количество разрядов данных
void sendCmd(IRsend *Sender, byte value, byte repeats, byte ndigits);

// Анализ и вывод результатов
// С отладочным выводом над быть очень осторожным: он занимает очень много времени
void OutputResultsIRA(void);
void OutputResultsRC5(void);
void OutputExecbuff(void);

void requestEvent();
void receiveEvent(int howMany);
//============================================================

unsigned long time_last_emit;

typedef struct  {
	byte stored_value;
	unsigned long time_val_last_upd;
  IRrecv tsop;
} receiver; 
receiver receivers[TSOP_NUM];

decode_results result;

// Итоговый результат для IRA. То, что выводится наружу.
byte Result[iralib::NTS] = {0,0,0,0};

//------------------------------------------------------------

#ifdef EMITTER
	IRsend sender(2);
#endif

// i2c buffer on receive:
// exec[0]	-	control cmd 
// exec[1]	-	number of arguments
// exec[2]	-	sending data
// exec[3]	-	frequency of emmiting
// exec[4]	-	extra byte (not used in current version)
byte exec[I2C_BUF_LEN];

// i2c buffer on request:
byte bufferMsg[I2C_BUF_LEN]; 

// Буфер приема RC5
unsigned long rc5recievedata[RC5Server::NUMRECV][2]; // 2 элемента использовались в другой версии
  // для хранения длинных сообщений из нескольких частей; в данной нужна лишь первая часть;
  // массив сохранён без изменений, чтобы не менять связанный код

byte I2Cdata[I2C_MAX_DATA];  // данные I2C
#ifdef DEBUG
bool msg_shown = false;
#endif

byte START_CODE = 0; //IRA

bool ira_mode;
// true если маяк в режиме псевдоаналоговой связи
// false - в режиме RC5

bool start=false; // флаг инициализации системы. Если true, то контроллер инициализирован и работает в конкретном режиме - RC5 или IRA
void setup()
{
  Serial.begin(BAUD_RATE);
	Wire.begin(ADDR);
	Wire.onReceive(receiveEvent);	
	Wire.onRequest(requestEvent);

#ifdef EMITTER
	Serial.print("IR emmiter addr ");
#else //RECEIVER
	receivers[0].tsop = IRrecv(0, PIN_TSOP_1);
	receivers[1].tsop = IRrecv(1, PIN_TSOP_2);
	receivers[2].tsop = IRrecv(2, PIN_TSOP_3);
	receivers[3].tsop = IRrecv(3, PIN_TSOP_4);
	Serial.print("IR Recv addr ");
#endif
	Serial.println(ADDR);

	unsigned long time_store = millis();
  while (not start){
    if (millis() - time_store > 5000)
    {
      Serial.println("Waiting for cmd to set mode");
      time_store = millis();
    }
  }

  if (exec[0] == CMD_SET_IRA_MODE)
  {
    ira_mode = true;
    memset(rc5recievedata, 0, sizeof(rc5recievedata));
    iralib::SetPNum(START_CODE);
    iralib::Init();
  #ifdef DEBUG
    Serial.println("Set IRA mode");
  #endif
  }
  else if (exec[0] == CMD_SET_RC5_MODE)
  {
    ira_mode = false;
    enableIRIn(); 					// Start the receiver 
  #ifdef DEBUG
    Serial.println("Set RC5 mode");
  #endif
  }

	time_last_emit = millis();
}

bool ira_emit_stoped = true; // был ли сигнал на излючение уже остановлен
unsigned long time_curr;

void loop()  
{
#ifdef EMITTER //EMITTER
  time_curr = millis();

  if (exec[0] == CMD_EMIT_STOP){ 
  #ifdef DEBUG
    if (not msg_shown) {
      Serial.println("No emition");
      msg_shown=true;
    }
  #endif
    if (ira_mode and not ira_emit_stoped)
      iralib::SetPNum(0);
  }
  else if (exec[0] == CMD_EMIT_START_RC5)
  {
    if (not ira_mode) 
    {
      if (exec[3]>0 and (time_curr - time_last_emit) > 1000/exec[3]){ 
        time_last_emit = time_curr;
        sendCmd(&sender, exec[2], REPETITION_DELAY, RC5numdigits);
    }
  #ifdef DEBUG
    if (msg_shown)		// reset flag
      msg_shown = false; 
  #endif
    }
  #ifdef DEBUG
    else
      Serial.println("RC5 mod is not set");
  #endif
  }
  else if (exec[0] == CMD_EMIT_START_IRA)
  {
    if (ira_mode) {
      iralib::SetPNum(exec[2]);
    }
    else
  #ifdef DEBUG
      Serial.println("IRA mod is not set");
  #endif

    ira_emit_stoped = false;
  }

#else //RECEIVER
	// ======= Upd sensor data =======
    if (ira_mode){
      iralib::Evaluate(Result);
      for(int i=0;i<RC5Server::NUMRECV;i++)
        rc5recievedata[i][0] = rc5recievedata[i][1] = Result[i]; // full package copy
    }
    else {
      for(byte i=0; i<TSOP_NUM; i++){
        time_curr = millis();
        if (receivers[i].tsop.decode(&result) and result.value>0){
          receivers[i].stored_value = result.value;
          receivers[i].time_val_last_upd = time_curr;
          receivers[i].tsop.resume(); // Receive the next value
        }
        else if ((time_curr - receivers[i].time_val_last_upd) > TSOP_KEEP_IN_MEMORY_TIME ) 
          receivers[i].stored_value = 0;		
      }
    }
#endif
}

void sendCmd(IRsend *Sender, byte value, byte repeats, byte ndigits)
{
#ifdef DEBUG
  Serial.print("emit:");
  Serial.println(value, HEX);
#endif

	unsigned long data;
	data = value;
	for(byte i=0; i<repeats; i++)
	{
		Sender->sendRC5(data, ndigits); 
		delay(REPETITION_DELAY);  		//According to the RC5 standard, there must be such repetition delay 
	}
} 

// on I2C request 
void requestEvent(){ 
	if(TSOP_NUM > I2C_BUF_LEN)
		Serial.println("Rcv > i2c buf");
	else {  

#ifndef EMITTER
		for(byte i=0; i<TSOP_NUM; i++)	
    {
      if (ira_mode)
        bufferMsg[i] = Result[i];
      else
        bufferMsg[i] = receivers[i].stored_value;
    }
#endif
    if (not start)
      bufferMsg[I2C_BUF_LEN-1]=NOT_START;
    else if (ira_mode)
      bufferMsg[I2C_BUF_LEN-1]=CMD_SET_IRA_MODE;
    else if (not ira_mode)
      bufferMsg[I2C_BUF_LEN-1]=CMD_SET_RC5_MODE;

		Wire.write(bufferMsg, I2C_BUF_LEN);
	}
}

// read on i2c
void receiveEvent(int howMany)
{    
	// read from i2c
	while (Wire.available()){
		for(byte i=0; i<I2C_BUF_LEN; i++)
			exec[i] = Wire.read();
	}
  if ((exec[0] == CMD_SET_IRA_MODE or exec[0] == CMD_SET_RC5_MODE) and not start)
      start=true;
}

void OutputResultsIRA(void)
{
  char buf[9];
  Serial.print("IRA: ");
  for(int i=0;i<RC5Server::NUMRECV;i++)
  {
    sprintf(buf, "%2d ", (byte)rc5recievedata[i][0]);
    Serial.print(buf);
  }
  Serial.println();
}

void OutputResultsRC5(void)
{
  Serial.print("RC5: ");
	for(byte i=0; i<TSOP_NUM; i++){
		Serial.print(receivers[i].stored_value, HEX);
		Serial.print('\t');
	}
	Serial.println();	
}

void OutputExecbuff(void)
{
	for(byte i=0; i<I2C_BUF_LEN; i++){
		Serial.print(exec[i], HEX);
		Serial.print('\t');
	}
	Serial.println();	
}
