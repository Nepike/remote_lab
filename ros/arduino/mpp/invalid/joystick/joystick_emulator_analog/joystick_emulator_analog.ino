/* 
 * Тестовая программа эмуляции джойстика с 
 * пультом ручного управления
 * 
 * @author Maxim Rovbo
 * @date 22.09.2016
 * @date LP 22.09.2016
 */
 
//
// Определение портов
//
// *** Выходы
// Управление скоростью
int PIN_X = 10; // поступательное
int PIN_Y = 9; // вращательное

// *** Входы
// Органы управления (аналоговые входы)
// Ручки газа (A0, A1)
int PIN_REG_X = A0;
int PIN_REG_Y = A1;

//-------------------------------------------------

double MIN_V = 1.1; // минимальное напряжение управляющего сигнала скорости,
                    // соответствующее 0 скорости
double MAX_V = 3.8; // максимальное напряжение управляющего сигнала скорости,
                    // соответствующее максимальной скорости

int MAX_ADC = 1024; // максимальное дискретное значение скорости
double V_to_ADC = 255 / 5.0; // перевод сигнала ручки в дискретные значения скорости

void setup() 
{
  // Изменение множителя счётчика на частоту 31 кГц на выходах 9, 10; 
  // (http://arduino-info.wikispaces.com/Arduino-PWM-Frequency) 
  TCCR1B = TCCR1B & B11111000 | B00000001;
  
  pinMode(PIN_X, OUTPUT);
  pinMode(PIN_Y, OUTPUT);
}

void loop() 
{
  int xspeed, yspeed;
  int xdata, ydata;
  
  // Считывание значений ручек скорости
  xdata = analogRead(PIN_REG_X);
  ydata = analogRead(PIN_REG_Y);

  // перевод значений ручек скоростей в напряжение на выходе для управления
  xspeed = (MIN_V + (MAX_V - MIN_V) * xdata / MAX_ADC) * V_to_ADC;
  yspeed = (MIN_V + (MAX_V - MIN_V) * ydata / MAX_ADC) * V_to_ADC;

  // отладочная печать переведённой скорости
  Serial.print(xspeed);
  Serial.print(", ");
  Serial.println(yspeed);
  
  // Выставление управляющего сигнала скорости
  analogWrite(PIN_X, xspeed);
  analogWrite(PIN_Y, yspeed);
  
  delay(5);
}
