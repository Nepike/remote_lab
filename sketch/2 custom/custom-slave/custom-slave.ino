#include <SoftwareSerial.h>
#include <NPK.h>
#include <LiquidCrystal_I2C.h>

SoftwareSerial espSerial(3, 2); // RX, TX;
extern LiquidCrystal_I2C lcd;
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
      DataPack response = pack.perform_command();
      uint8_t* data = response.get_serialized_data();
      size_t size = response.get_serialized_size();
      if (response.is_valid() && response.get_args_count()) {
        // Serial.println(response.get_next_val<const char*>());
        espSerial.write(data, size);
        espSerial.flush();
      }
      
      free(data);
    }
  }
}