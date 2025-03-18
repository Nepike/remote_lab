#include <SoftwareSerial.h>
#include <NPK.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>


SoftwareSerial espSerial(3, 2); // RX, TX;
LiquidCrystal_I2C lcd(0x27, 16, 2);

MPU6050 mpu(0x68);
// PID-параметры
float Kp = 1.5, Ki = 0.1, Kd = 0.3;
float error = 0, integral = 0, prevError = 0;
unsigned long lastTime = 0;

int IN1 = 9;
int IN2 = 8;
int IN3 = 7; 
int IN4 = 6; 
int EN1 = 10; 
int EN2 = 5;

int LE1 = 11;
int LE2 = 12;
int RE1 = 4;
int RE2 = 13;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;




DataPack DataPack::perform_command() {
	DataPack response(Command::RESPONSE);

	if (!is_valid()) {
		response.is_valid_ = false;
		return response;
	}

	switch(command_){
		case Command::RESPONSE: {
			response.is_valid_ = false;
			return response;
		}
		// case Command::EXAMPLE: {
		//   //Функия return_type example(type1 arg1, type2 arg2, ...) - есть в поле видимости и имеет соответствующий индекс в Command
		//   // Из-за невозмжности отключить NRVO придется явно сохранять возвращаемые значения в переменных: 
		//   auto arg1 = get_next_val<type1>();
		//   auto arg2 = get_next_val<type2>();
		//   ...
		//   return_type return_value = example(arg1, arg2, ...);
		//   // Заполнение response
		//   return response;
		// }
		case Command::BLINK: {
			digitalWrite(LED_BUILTIN, HIGH);
			delay(1000);
			digitalWrite(LED_BUILTIN, LOW);
			delay(1000);
			break;
		}
		case Command::LCD: {
			lcd.clear();
			lcd.setCursor(0,0);
			auto arg = get_next_val<const char*>();
			lcd.print(arg);
			break;
		}
    case Command::GETSPEED: {
      uint8_t encoder = get_next_val<uint8_t>();
			long count;
      noInterrupts();
      count = (encoder == 0) ? encoder1Count : 
              (encoder == 1) ? encoder2Count : -1;
      interrupts();
      response.append_arg(String(count).c_str());
      break;
		}
		case Command::GETRANGE: {
			int max_length = 0; //надо померить
			int min_length = 1023; //надо померить
			int S1 = 6;
			int sig = analogRead(S1);
			sig = map(sig, 0, 1023,max_length, min_length);
			String s = String(sig);
			response.append_arg(s.c_str());
			break;
		}
		case Command::GO: {
			uint8_t direction = get_next_val<uint8_t>();
			uint8_t speed = get_next_val<uint8_t>();
			uint16_t time = get_next_val<uint16_t>();

			analogWrite (EN1, 0); 
			analogWrite (EN2, 0);
			if (time == 0) {
				break;
			}

			switch(direction) {
				case 1 : { // forward
					digitalWrite (IN2, HIGH); 
					digitalWrite (IN1, LOW); 
					digitalWrite (IN4, HIGH); 
					digitalWrite (IN3, LOW);
					break;
				}
        case 2: { // right
					digitalWrite (IN2, HIGH); 
					digitalWrite (IN1, LOW); 
					digitalWrite (IN3, HIGH); 
					digitalWrite (IN4, LOW);
					break;
				}
				case 3: { // back
					digitalWrite (IN1, HIGH); 
					digitalWrite (IN2, LOW); 
					digitalWrite (IN3, HIGH); 
					digitalWrite (IN4, LOW);
					break;
				}
				case 4: { // left
					digitalWrite (IN1, HIGH); 
					digitalWrite (IN2, LOW); 
					digitalWrite (IN4, HIGH); 
					digitalWrite (IN3, LOW);
					break;
				}
				default: {return response;}
			}

			analogWrite(EN1, speed); 
			analogWrite(EN2, speed);

			// Основной цикл управления
      unsigned long startTime = millis();
      while(true) {
        // Вычисление времени между итерациями
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        lastTime = now;

        // Получаем данные с гироскопа
        int16_t gz = mpu.getRotationZ();
        float rotationRate = gz / 131.0; // Перевод в °/с

        // PID-расчет только для прямолинейного движения
        if(direction == 1 || direction == 3) {
          error = -rotationRate; // Цель - нулевая угловая скорость
          integral += error * dt;
          float derivative = (error - prevError) / dt;
            
          // Вычисление корректирующего сигнала
          float correction = Kp*error + Ki*integral + Kd*derivative;
            
          // Применяем коррекцию к моторам
          analogWrite(EN1, constrain(speed + correction, 0, 255));
          analogWrite(EN2, constrain(speed - correction, 0, 255));
          
          prevError = error;
        } 
        else {
          // Для поворотов обычное управление
          analogWrite(EN1, speed);
          analogWrite(EN2, speed);
        }

        // Проверка времени
        if(time != 1 && (millis() - startTime) > time) {
          break;
        }
      
        // Задержка для стабильной работы (10 мс ~ 100 Гц)
        delay(10);
      }
    
      analogWrite(EN1, 0);
      analogWrite(EN2, 0);
      break;
		}
		default: {
			return response;
		};
	}
	return response;
}


void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  lcd.init();
  lcd.backlight();
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(EN2, OUTPUT); 
  pinMode(IN4, OUTPUT); 
  pinMode(IN3, OUTPUT); 

  mpu.initialize();
  delay(200);
  mpu.CalibrateGyro();

  // Настройка пинов для первого энкодера (левый)
  pinMode(LE1, INPUT_PULLUP);
  pinMode(LE2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LE1), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LE2), encoder1ISR, CHANGE);

  // Настройка пинов для второго энкодера (правый)
  pinMode(RE1, INPUT_PULLUP);
  pinMode(RE2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RE1), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RE2), encoder2ISR, CHANGE);

}

// Обработчик для первого энкодера
void encoder1ISR() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(encoder1Count++);
  // static uint8_t oldState = 0;
  // uint8_t newState = (digitalRead(LE1) << 1) | digitalRead(LE2);
  // uint8_t stateChange = (oldState << 2) | newState;

  // if (stateChange == 0b0001 || stateChange == 0b0111 || stateChange == 0b1110 || stateChange == 0b1000) {
  //   encoder1Count++;
  // } else if (stateChange == 0b0010 || stateChange == 0b1011 || stateChange == 0b1101 || stateChange == 0b0100) {
  //   encoder1Count--;
  // }
  // oldState = newState;
}

// Обработчик для второго энкодера
void encoder2ISR() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(encoder2Count++);
  // static uint8_t oldState = 0;
  // uint8_t newState = (digitalRead(RE1) << 1) | digitalRead(RE2);
  // uint8_t stateChange = (oldState << 2) | newState;

  // if (stateChange == 0b0001 || stateChange == 0b0111 || stateChange == 0b1110 || stateChange == 0b1000) {
  //   encoder2Count++;
  // } else if (stateChange == 0b0010 || stateChange == 0b1011 || stateChange == 0b1101 || stateChange == 0b0100) {
  //   encoder2Count--;
  // }
  // oldState = newState;
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