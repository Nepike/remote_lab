#include "PololuMaestro.h"
 
MiniMaestro maestro(Serial);
 
void setup()
{
  // Set the serial port's baud rate.
  Serial.begin(9600);
}
 
void loop()
{
  /* setTarget takes the channel number you want to control, and
     the target position in units of 1/4 microseconds. A typical
     RC hobby servo responds to pulses between 1 ms (4000) and 2
     ms (8000). */
 
  // Set the target of channel 0 to 1500 us and channel 1 to 1750 us.
  maestro.setTarget(0, 6000);
  maestro.setTarget(1, 7000);
 
  // Wait 2 seconds.
  delay(2000);
 
  // Set the target of channel 0 to 1250 us and channel 1 to 2000 us.
  maestro.setTarget(0, 5000);
  maestro.setTarget(1, 8000);
 
  // Wait 2 seconds.
  delay(2000);
}
