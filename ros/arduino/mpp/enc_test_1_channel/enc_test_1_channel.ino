
// Encoders
#define PIN_L 2
#define PIN_R 3

volatile int
  encoderLeftCnt = 0,
  encoderRightCnt = 0;
  
/**
 * Системная функция. Обработчик прерывания для энкодеров
 * Не совсем корректно работает: регистрируются изменения сигнала (переходы с 0 на 1 и с 1 на 0),
 * а не сам сигнал, поэтому показания могут быть в 2 раза выше, чем на самом деле
 */
void doEncoder(void)
{
  static byte pred_eLeft = 0;
  static byte pred_eRight = 0;
  byte eLeft = digitalRead(PIN_L);
  byte eRight = digitalRead(PIN_R);
  if(eLeft != pred_eLeft)
  {
    pred_eLeft = eLeft;
    encoderLeftCnt++;
  }
  if(eRight != pred_eRight)
  {
    pred_eRight = eRight;
    encoderRightCnt++;
  }
};

void setup()
{
  Serial.begin(115200);
  
  // Настройка энкодеров
  pinMode(PIN_L, INPUT);
  digitalWrite(PIN_L, HIGH);    // turn on pullup resistor

  pinMode(PIN_R, INPUT);
  digitalWrite(PIN_R, HIGH);   // turn on pullup resistor

  
  // Настройка прерываний для энкодеров
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 1 - pin 3
}

void loop()
{
  Serial.print(encoderLeftCnt);
  Serial.print(",");
  Serial.print(encoderRightCnt);
  Serial.print("\n");

}
