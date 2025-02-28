#include <SoftwareSerial.h>
#include <NPK.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial espSerial(3, 2); // RX, TX
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  lcd.init();
  lcd.backlight();
}

void loop() {
  if (espSerial.available() >= 2) {
    DataPack pack(espSerial);
    
    if (pack.is_valid()) {
      Serial.println("\nValid packet received!");
      pack.perform_command();
    }
  }
}