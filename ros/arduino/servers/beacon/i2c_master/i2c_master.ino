// Отладочный i2c сервер для работы с маяком
#include <Wire.h>

#define I2C_BUF_LEN 5
#define ADDR_RCV	0x80
#define ADDR_EMIT 	0x70
#define BAUD_RATE 9600

// cmds
#define CMD_EMIT_STOP 	    0
#define CMD_EMIT_START_RC5  1
#define CMD_EMIT_START_IRA 	2
#define CMD_SET_IRA_MODE   	3
#define CMD_SET_RC5_MODE   	4

#define FREQ 5
#define DELAY_BEETWEN_PKG_SEND 5000 // in ms
#define DELAY_BEETWEN_STOP_PKG_SEND 15000 // in ms
byte buf[I2C_BUF_LEN]; // буфер обмена по i2c
byte cmd;
void setup() {
	Serial.begin(BAUD_RATE);
	Serial.println(" i2c master operates");
	Serial.print("receiver addr: ");
	Serial.print(ADDR_RCV);
	Serial.print("\temitter addr: ");
	Serial.println(ADDR_EMIT);
	Wire.begin(); 

  buf[1]=3;    // num of args

  /* ====== set mode package for beacon controller ===== */
	/* buf[0] = CMD_EMIT_STOP; */
	/* buf[0] = CMD_SET_RC5_MODE; */
	buf[0] = CMD_SET_IRA_MODE;
	Wire.beginTransmission(ADDR_RCV); 
	Wire.write(buf, I2C_BUF_LEN);       
	Wire.endTransmission();
	Wire.beginTransmission(ADDR_EMIT); 
	Wire.write(buf, I2C_BUF_LEN);       
	Wire.endTransmission();
}

unsigned long t1=millis();
unsigned long t2=millis();
void loop() {
	/* buf[0] = CMD_EMIT_START_RC5; */
	buf[0] = CMD_EMIT_START_IRA;
	buf[2] = random(7,10);
  buf[3] = FREQ;

  if (millis()-t1>DELAY_BEETWEN_PKG_SEND)
  {
    if (millis()-t2>DELAY_BEETWEN_STOP_PKG_SEND)
    {
      buf[0] = CMD_EMIT_STOP;
      t2=millis();
    }
    Wire.beginTransmission(ADDR_RCV); 
    Wire.write(buf, I2C_BUF_LEN);       
    Wire.endTransmission();

    Wire.beginTransmission(ADDR_EMIT); 
    Wire.write(buf, I2C_BUF_LEN);       
    Wire.endTransmission();

    // =====	request data from slave	=====

    Serial.print("\nsend:\t");
    for(byte i=0; i<I2C_BUF_LEN; i++){
      Serial.print(buf[i], HEX);
      Serial.print(' ');
    }
    delay(500);
    t1=millis();
  }

	Serial.print("\nreceive:\t");
	Wire.requestFrom(ADDR_RCV, I2C_BUF_LEN);    
	while (Wire.available()) { 
		byte c = Wire.read(); 
		Serial.print(c, HEX);
		Serial.print(' ');
	}
	Wire.requestFrom(ADDR_EMIT, I2C_BUF_LEN);    
	while (Wire.available()) { 
		byte c = Wire.read(); 
		Serial.print(c, HEX);
		Serial.print(' ');
	}
	

}
