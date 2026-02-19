#include <Wire.h>
#include "Encoder.h"

// Encoders

#define PIN_E_R_1 2
#define PIN_E_R_2 3
#define PIN_E_L_1 4
#define PIN_E_L_2 5

Encoder encoder_right( PIN_E_R_1, PIN_E_R_2 );
Encoder encoder_left( PIN_E_L_1, PIN_E_L_2 );

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  int er = encoder_right.read();
  int el = encoder_left.read();

  Serial.print(er);
  Serial.print(",");
  Serial.print(el);
  Serial.print(",0\n");

}
