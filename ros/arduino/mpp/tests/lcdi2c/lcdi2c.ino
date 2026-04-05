/*
  Тест LCD-терминала с I2C
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// set the LCD address to 0x20 (Cooperate with 3 short circuit caps) for a 16 chars and 2 line display
// Arduino UNO: connect SDA to pin A4 and SCL to pin A5 on your Arduino
LiquidCrystal_I2C lcd(0x27,16,2); // 0x27

#define Title "LCD I2C Test"
#define PIN_ADC 0

void setup() 
{
  Serial.begin(9600);
  
  Serial.println("lcd.init...");
  // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.home();

  Serial.println("lcd.setCursor...");
  // устанавливаем курсор в колонку 0, строку 1. Нумерация начинается с нуля.
  lcd.setCursor(0, 0);
  // печатаем первую строку
  Serial.println("lcd.print...");
  lcd.print(Title);
}

char s[20];

void loop() 
{
  int n;
  float v;
  int vint, vfrac;
  
  n = analogRead(PIN_ADC);
  v = n*5.0/1023;
  vint = (int)v;
  vfrac = (v - vint) * 100;
    
  sprintf(s,"%4d  %1d.%2d ", n, vint, vfrac);
  lcd.setCursor(0, 1);
  lcd.print(s);

  Serial.println(s);

  delay(100);
}
