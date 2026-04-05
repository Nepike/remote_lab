/*
 * I2C Master. Simple Transmitter
 *
 * V 1.02
 * 11.03.2015
 *
 */

#include <Wire.h>
#include <i2cwctl.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("I2C Master Demo 1.02");

  delay(2000);
  lpwSendCommand(I2C_LPW_CTL_ADDR, I2C_CMD_CALIBRATE, 0);  
  delay(5000);
}

#define angnum 11

signed char angles[angnum] = {0, -30, -60, -30, 0, 30, 60, 30, 0, -110, 110};

void loop()
{
  byte i;
/*
  Serial.println("Speed = 100");
  lpwSendCommand(I2C_LPW_CTL_ADDR, I2C_CMD_SET_SPEED, 100);  
  for(i=0;i<angnum;i++)
  {
    Serial.println((int)angles[i]);
    lpwSendCommand(I2C_LPW_CTL_ADDR, I2C_CMD_SET_ANG, (int)angles[i]);
    delay(3000);
  }
*/
  Serial.println("Speed = 100");
  lpwSendCommand(I2C_LPW_CTL_ADDR, I2C_CMD_SET_SPEED, 100);  
  for(i=0;i<angnum;i++)
  {
    Serial.println((int)angles[i]);
    lpwSendCommand2B(I2C_LPW_CTL_ADDR, I2C_CMD_SET_ANG, angles[i], 0);
    delay(3000);
  }
}

