
/*
  HMC5883L Triple Axis Digital Compass. Output for HMC5883L_processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <HMC5883L.h>
#include <EEPROM.h>

#define EEPROM_ADDR_OFFSET_X 0
#define EEPROM_ADDR_OFFSET_Y 1

#define CALIB_PIN_BUTTON 3
#define CALIB_PIN_LED 4

HMC5883L compass;
bool state = true; // true = measure, false = calibrate

void setup()
{
  Serial.begin(115200);

  pinMode(CALIB_PIN_BUTTON, INPUT_PULLUP);
  pinMode(CALIB_PIN_LED, OUTPUT);

  // Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  attachInterrupt(digitalPinToInterrupt(CALIB_PIN_BUTTON), changeState, CHANGE);

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset 
  compass.setOffset( EEPROM.read(EEPROM_ADDR_OFFSET_X), EEPROM.read(EEPROM_ADDR_OFFSET_Y) ); 
}

void changeState()
{
  state = !state;
}

void loop()
{
  if(!state)
  {
    Serial.println("Started Calibration Mode, rotate platform 2-3 times on 2PI");
    digitalWrite(CALIB_PIN_LED,HIGH);    
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int offX = 0;
    int offY = 0;
    while(!state)
    {
      Vector mag = compass.readRaw();    
      
      // Determine Min / Max values
      if (mag.XAxis < minX) minX = mag.XAxis;
      if (mag.XAxis > maxX) maxX = mag.XAxis;
      if (mag.YAxis < minY) minY = mag.YAxis;
      if (mag.YAxis > maxY) maxY = mag.YAxis;
    
      // Calculate offsets
      offX = (maxX + minX)/2;
      offY = (maxY + minY)/2;
    }
    Serial.print("Exit Calibration Mode, offsets = ");
    Serial.print(offX);
    Serial.print(" ");
    Serial.println(offY);
    
    digitalWrite(CALIB_PIN_LED,LOW);
    EEPROM.write(EEPROM_ADDR_OFFSET_X,offX);
    EEPROM.write(EEPROM_ADDR_OFFSET_Y,offY);
    compass.setOffset(offX, offY);
  }

  
  long x = micros();
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
 
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  
  // Output
  Serial.print("-1, -1, ");
  Serial.println(-heading);
  
  // One loop: ~5ms @ 115200 serial.
  // We need delay ~28ms for allow data rate 30Hz (~33ms)
  delay(30);
}
