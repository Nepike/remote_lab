/*
 * Программа контроллера пульта ручного управления МПП
 * 
 * @author Maxim Rovbo
 * @date 22.08.2016
 * @date LP 01.09.2016
 */

//
// Определение портов
//
// *** Выходы

// Управление скоростью
int PIN_GAZ_LEFT = 10;
int PIN_GAZ_RIGHT = 9;

// Останов
int PIN_STOP_LEFT = 8;
int PIN_STOP_RIGHT = 4;

// Реверс
int PIN_REVERSE_LEFT = 7;
int PIN_REVERSE_RIGHT = 3;

// *** Входы
// Органы управления (аналоговые входы)
// Ручки газа (A0, A1)
int PIN_REG_GAZ_LEFT = A0;
int PIN_REG_GAZ_RIGHT = A1;

// Аналоговые входы, которые будут использоваться в цифровом режиме
// Клавиши "стоп"
int PIN_BTN_STOP_LEFT = A2;
int PIN_BTN_STOP_RIGHT = A3;

int PIN_BTN_REVERSE_LEFT = A4;
int PIN_BTN_REVERSE_RIGHT = A5;

//-------------------------------------------------

double MIN_V = 1.0; // минимальное напряжение управляющего сигнала скорости,
                    // соответствующее 0 скорости
double MAX_V = 2.5; // максимальное напряжение управляющего сигнала скорости,
                    // соответствующее максимальной скорости

int MAX_ADC = 1024; // максимальное дискретное значение скорости
double V_to_ADC = 255 / 5.0; // перевод сигнала ручки в дискретные значения скорости

void setup()
{
  Serial.begin(115200);
  
  // Изменение множителя счётчика на частоту 31 кГц на выходах 9, 10; 
  // (http://arduino-info.wikispaces.com/Arduino-PWM-Frequency) 
  TCCR1B = TCCR1B & B11111000 | B00000001;
  /*
  изменение множителя счётчика на частоту 31 кГц на выходах 3, 11; 
  TCCR2B = TCCR2B & B11111000 | B00000001;
  */
  
  // Входы
  pinMode(PIN_BTN_STOP_LEFT, INPUT_PULLUP);  
  pinMode(PIN_BTN_STOP_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_REVERSE_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_REVERSE_RIGHT, INPUT_PULLUP);

  // На всякий случай:
  digitalWrite(PIN_BTN_STOP_LEFT, HIGH);
  digitalWrite(PIN_BTN_STOP_RIGHT, HIGH);
  digitalWrite(PIN_BTN_REVERSE_LEFT, HIGH);
  digitalWrite(PIN_BTN_REVERSE_RIGHT, HIGH);

  // Выходы
  pinMode(PIN_GAZ_LEFT, OUTPUT);
  pinMode(PIN_GAZ_RIGHT, OUTPUT);

  pinMode(PIN_STOP_LEFT, OUTPUT);
  pinMode(PIN_STOP_RIGHT, OUTPUT);

  pinMode(PIN_REVERSE_LEFT, OUTPUT);
  pinMode(PIN_REVERSE_RIGHT, OUTPUT);
}

void loop()
{
  int lspeed, rspeed;
  int ldata, rdata;
  int button_brakes_0, button_brakes_1;
  int button_reverse_0, button_reverse_1;

  // Считывание кнопок тормозов
  button_brakes_0 = digitalRead(PIN_BTN_STOP_LEFT);
  button_brakes_1 = digitalRead(PIN_BTN_STOP_RIGHT);

  // Считывание кнопок реверса
  button_reverse_0 = digitalRead(PIN_BTN_REVERSE_LEFT);
  button_reverse_1 = digitalRead(PIN_BTN_REVERSE_RIGHT);

  // Считывание значений ручек скорости
  ldata = analogRead(PIN_REG_GAZ_LEFT);
  rdata = analogRead(PIN_REG_GAZ_RIGHT);

  // перевод значений ручек скоростей в напряжение на выходе для управления
  lspeed = (MIN_V + (MAX_V - MIN_V) * ldata / MAX_ADC) * V_to_ADC;
  rspeed = (MIN_V + (MAX_V - MIN_V) * rdata / MAX_ADC) * V_to_ADC;

  // отладочная печать скорости
  Serial.print(ldata);
  Serial.print(", ");
  Serial.print(rdata);

  // отладочная печать входных значений кнопок и переведённой скорости
  Serial.print(", ");
  Serial.print(lspeed);
  Serial.print(", ");
  Serial.print(rspeed);
  Serial.print(", ");
  Serial.print(button_brakes_0);
  Serial.print(", ");
  Serial.print(button_brakes_1);
  Serial.print(", ");
  Serial.print(button_reverse_0);
  Serial.print(", ");
  Serial.print(button_reverse_1);
  Serial.println();

  // Выставление управляющего сигнала скорости
  analogWrite(PIN_GAZ_LEFT, lspeed);
  analogWrite(PIN_GAZ_RIGHT, rspeed);

  // Выставление сигналов кнопок
  digitalWrite(PIN_STOP_LEFT, button_brakes_0);
  digitalWrite(PIN_STOP_RIGHT, button_brakes_1);
  digitalWrite(PIN_REVERSE_LEFT, button_reverse_0);
  digitalWrite(PIN_REVERSE_RIGHT, button_reverse_1);

  delay(5);
}
