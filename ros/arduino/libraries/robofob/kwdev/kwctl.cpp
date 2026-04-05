/*
 * 
 * 22.08.2018
 * LP 13.11.2022
 */
 
#include <Arduino.h>
#include "kwctl.h"

int kwc::eeAddress = 0;

kwc::TProgram kwc::Program;

long kwc::GlobalProgCounter = 0;

byte kwc::debug_regime = 0;
boolean kwc::was_cmd_debug_regime = false;
int kwc::GLOBAL_PWM_LIMIT = 0;

// Масштабный множитель для всех значений амплитуд двигательных программ
float kwc::progr_scale_A = 1;
// Масштабный множитель для частот двигательных программ
float kwc::progr_scale_F = 1;

// Порты
Stream *kwc::PCStream;

//------------------------------------------------------------------------------
//
// Сенсоры
//
//------------------------------------------------------------------------------
// Массив значений сенсоров
int kwc::sData[_kwc_NUM_SENS];

TRBuffer<int, 4> Sens[_kwc_NUM_SENS];

const int kwc::MAX_US_DISTANCE = 500;

//------------------------------------------------------------------------------
//
// Эффекторы
//
//------------------------------------------------------------------------------
TDevice *kwc::dev[_kwc_NUM_DEV];

//------------------------------------------------------------------------------

String kwc::MReadStr(char c)
{
  while(!PCStream->available());
  String s = PCStream->readStringUntil(c);
  return s;
}

void kwc::MReadStr(char *res, char c)
{
  String s = MReadStr(c);
  strcpy(res, s.c_str());
}

int kwc::MReadInt(void)
{
  while(!PCStream->available());
  int n = PCStream->parseInt();
  return n;
}

void kwc::LCDInit(char *title)
{
  lcd.begin();
  lcd.backlight();
  lcd.home();
  LCDprint(0, title);
}

void kwc::LCDprint(const char *s) { lcd.print(s); }
void kwc::LCDprint(byte b) { lcd.print(b); }
void kwc::LCDprint(byte line, const char *s) { lcd.setCursor(0, line); LCDprint(s);}
void kwc::LCDprint(byte line, byte row, char *s) { lcd.setCursor(row, line); LCDprint(s); }
void kwc::LCDprint(byte line, byte row, byte b) { lcd.setCursor(row, line); LCDprint(b); }
void kwc::LCDprint(byte line, byte b) { lcd.setCursor(0, line); lcd.print(b); }

void kwc::Beep(byte n)
{
  for(byte i=0;i<n;i++)
  {
    digitalWrite(PIN_BEEP, HIGH);
    delay(200);
    digitalWrite(PIN_BEEP, LOW);
    delay(200);
  }
}

void kwc::ProgramInit(void)
{
  strcpy(Program.pname,"XXX");
  memset(Program.data, 0, sizeof(Program.data));
  for(byte n=0;n<_kwc_NUM_DEV;n++)
  {
    sprintf(Program.description[n], "p %02d", n);
    // У актуаторов шкала времени растянута
    if(n<_kwc_NUM_ACTUATORS)
      Program.ratio[n] = 10;
    else
      Program.ratio[n] = 1;
    Program.enabled[n] = 0;
    Program.tcnt[n] = 0;
  }
}

/*
 * Таймер
 */
void kwc::timerIsr()
{
  kwdev::T_WCNT++;
  static byte n = 0;
  n++;
  if(n>=2*(6-progr_scale_F)) // (n>=10) progr_scale_F=[1..5]
  {
    n = 0;
    GlobalProgCounter++;
  }
}

/*
 * Инициализация всех устройств
 *   70, 170 - China long
 *    9, 250 - China extra long
 * Китайские короткие:
 *   80(82)-170(179)
 * SKF короткие:
 *   14(15)-150(150)
 */

int kwc::InitDevices(void)
{
  kwdev::SMC.Init();

  char name[10];
  // name, addr, pin_adc, minr, maxr, minctl, maxctl
  int ndev = 0;
  for(byte i=0;i<_kwc_NUM_ACTUATORS;i++)
  {
    sprintf(name,"act%02d",ndev);
    /*
    if(ndev<=3)
      dev[ndev] = new TActuator(name, i+1, PIN_ACT_ADC[i], 70, 170, 0, 100); // China long
    else
      dev[ndev] = new TActuator(name, i+1, PIN_ACT_ADC[i],  9, 250, 0, 100); // China extra long
    */
    dev[ndev] = new TActuator(name, i+1, PIN_ACT_ADC[i],  9, 250, 0, 100); // China extra long
    ndev++;
  }

  for(byte i=0;i<_kwc_NUM_298MOTORS;i++)
  {
    sprintf(name,"mtr%02d",ndev);
    dev[ndev] = new TL298(name, PIN_M_DIR[i], PIN_M_PWM[i], -100, 100);
    ndev++;
  }

  for(byte i=0;i<_kwc_NUM_SERVO;i++)
  {
    sprintf(name,"srv%02d",ndev);
    dev[ndev] = new TServo(name, i, -90, 90);
    ndev++;
  }

  for(byte i=0;i<_kwc_NUM_PWM_DEV;i++)
  {
    sprintf(name,"pwm%02d",ndev);
    dev[ndev] = new TPWMDevice(name, PIN_PWM_DEV[i],  0, 100);
    ndev++;
  }

  for(byte i=0;i<_kwc_NUM_DEV;i++)
    dev[i]->Enable();
  return ndev;
}

void kwc::ResetAll(void)
// Сброс всех устройств (драйверов)
{
  Beep(1);
  for(byte i=0;i<_kwc_NUM_DEV;i++)
  {
    dev[i]->Reset();
    delay(20);
  }
}

/*
 * Инициализация программы и запись ее в EEPROM
 * Должна вызываться один раз при прошивке контроллера
 * См. boolean kwc::FirstRun
 */
void kwc::ResetDeviceProgram(void)
{
  ProgramInit();
  EEPROM.put(eeAddress, Program);
}

/**
 *  _pc_stream - Computer (9600)
 * _motor_stream - Actuators (Pololu motor drivers) (19200)
 * _servo_stream - Servo (Pololu Maestro) (9600)
 *
 */
void kwc::InitSystem(char *title, Stream &_pc_stream, Stream &_motor_stream, Stream &_servo_stream)
{
  PCStream = &_pc_stream;
  kwdev::MotorStream = &_motor_stream;
  kwdev::ServoStream = &_servo_stream;
  // Инициализация lcd
  LCDInit(title);

  //----------------------------------------------------------
  // Настройка портов
  pinMode(PIN_BEEP, OUTPUT);
  pinMode(PIN_REGIME_SWITCH, INPUT_PULLUP);
  pinMode(PIN_DEBUG_REG_INDICATOR, OUTPUT);

  for(byte i=0;i<_kwc_NUM_ANALOG_SENS;i++)
    pinMode(PIN_ANALOG_SENS[i], INPUT);

  //----------------------------------------------------------
  delay(100);

  // Таймер
  // Пример: tns = 50000: таймер будет прерываться с периодом 50'000 микросекунд (или 0.05 сек - или 20Hz)
  long tns = 100000L; // 10Гц
  Timer1.initialize(tns);
  Timer1.attachInterrupt(timerIsr); // attach the service routine here

  InitDevices();

  if(FirstRun)
  {
    LCDprint(0, 14, "FR!");
    // Сброс программы
    ResetDeviceProgram();
  }
  else
  {
    LCDprint(0, 14, "   ");
    // Пробуем вытащить из EEPROM
    EEPROM.get(eeAddress, Program);
  }

  while(PCStream->available()) PCStream->read();
  Beep(3);
}

void kwc::PrintSensors(void)
{
  PCStream->print("> ");
  PCStream->print(_kwc_NUM_SENS);
  PCStream->print(' ');
  for(int i=0; i<_kwc_NUM_SENS; i++)
  {
    PCStream->print(sData[i], DEC);
    PCStream->print(' ');
  }
  PCStream->println('.');
}

// Показать статус устройств (скорости и проч)
void kwc::ShowStatus(void)
{
  PCStream->print("= ");
  PCStream->print(_kwc_NUM_DEV);
  PCStream->print(' ');
  for(byte i=0;i<_kwc_NUM_DEV;i++)
  {
    dev[i]->GetStatus();
    PCStream->print(kwdev::sline);
  }
  PCStream->println('.');
}

void kwc::ReadSensors(void)
{
  // get distance
  static byte cn = 0;
  int ds = 0;

  ds = US_SENSORS[cn].Ranging(CM);
  if(ds==0 || ds>MAX_US_DISTANCE) ds = MAX_US_DISTANCE;
  Sens[cn].Add(ds);
  cn++;
  if(cn>=_kwc_NUM_US_SENS) cn = 0;

  for(int i=0;i<_kwc_NUM_ANALOG_SENS;i++)
    Sens[i+_kwc_NUM_US_SENS].Add(analogRead(PIN_ANALOG_SENS[i]));

  for(byte i=0;i<_kwc_NUM_SENS;i++)
    sData[i] = Sens[i].GetVal();

}

void kwc::ProgramWrite(void)
{
  LCDprint(0, "Write...");
  PCStream->print(" #");
  PCStream->print(Program.pname);
  PCStream->println('#');
  PCStream->println(_kwc_NUM_DEV);
  for(byte i=0; i<_kwc_NUM_DEV;i++)
  {
    PCStream->print(dev[i]->name);
    PCStream->println("# ");

    PCStream->print(Program.description[i]);
    PCStream->println("# ");

    PCStream->print(Program.enabled[i]);
    PCStream->println(' ');

    PCStream->print(dev[i]->min_bound);
    PCStream->print(' ');
    PCStream->print(dev[i]->max_bound);
    PCStream->println();
    PCStream->println(PROG_LEN);
    PCStream->println(Program.ratio[i]);
    for(byte j=0; j<PROG_LEN;j++)
    {
      PCStream->print(Program.data[i][j]);
      PCStream->print(' ');
    }
    PCStream->println();
  }
  PCStream->println('#');
  LCDprint(0, "Done    ");
}

void kwc::ProgramRead(void)
{
  String s;
  LCDprint(0, "Read...   ");

  // Пропускаем лишнее
  MReadStr(kwdev::sline, '#');

  // read ProgramName
  MReadStr(kwdev::sline, '#');
  kwdev::sline[STR_LEN-1] = 0;
  strcpy(Program.pname, kwdev::sline);

  // read num_dev (ignore)
  int n = MReadInt();

  for(byte i=0; i<_kwc_NUM_DEV;i++)
  {
    // read dev name (ignore)
    s = MReadStr('#');

    // read dev rem
    MReadStr(kwdev::sline, '#');
    kwdev::sline[STR_LEN-1] = 0;
    strcpy(Program.description[i], kwdev::sline);

    Program.enabled[i] = MReadInt();

    // read bounds (ignore)
    n = MReadInt();
    n = MReadInt();

    // read prog len(ignore)
    n = MReadInt();

    // read prog ratio
    Program.ratio[i] = MReadInt();

    // read program
    for(byte j=0; j<PROG_LEN;j++)
    {
      n = MReadInt();
      Program.data[i][j] = n;
      sprintf(kwdev::sline, "%d %d %d  ", i, j, n);
      LCDprint(1, kwdev::sline);
    }
  }
  LCDprint(0, "Done    ");
}


/*
 * Чтение и отработка внешней команды
 */
void kwc::EvalExtCmd(void)
{
  if(PCStream->available())
  {
    String cmd = PCStream->readStringUntil(' ');

    if(cmd=="resetall")
    {
      ResetAll();
    }
    if(cmd=="reset")
    {
      int num = PCStream->parseInt();
      if(num>=0 && num<_kwc_NUM_DEV)
        dev[num]->Reset();
    }
    if(cmd=="set")
    {
      int num = PCStream->parseInt();
      int val = PCStream->parseInt();
      if(num>=0 && num<_kwc_NUM_DEV)
      {
        // Ограничения на значения для PWM-устройств
        if((num>=_kwc_NUM_DEV-_kwc_NUM_PWM_DEV) && (val>GLOBAL_PWM_LIMIT)) val = GLOBAL_PWM_LIMIT;
        dev[num]->SetGoal(val);
        sprintf(kwdev::sline, "set %2d %4d", num, val);
        LCDprint(1, kwdev::sline);
      }
    }
    if(cmd=="dctl")
    {
      int num = PCStream->parseInt();
      int val = PCStream->parseInt();
      if(num>=0 && num<_kwc_NUM_DEV)
      {
        dev[num]->_dctl(val);
        sprintf(kwdev::sline, "dctl %2d %4d", num, val);
        LCDprint(1, kwdev::sline);
      }
    }
    if(cmd=="info")
    {
      int num = PCStream->parseInt();
      if(num>=0 && num<_kwc_NUM_DEV)
      {
        dev[num]->GetStatus();
        PCStream->println(kwdev::sline);
        sprintf(kwdev::sline, "info %2d     ", num);
        LCDprint(1, kwdev::sline);
      }
    }
    if(cmd=="getsens")
    {
      PrintSensors();
    }
    if(cmd=="getstat")
    {
      ShowStatus();
    }
    if(cmd=="read")
    {
      ProgramWrite();
    }
    if(cmd=="save")
    {
      ProgramRead();
      EEPROM.put(eeAddress, Program);
    }
    if(cmd=="resetdevice")
    {
      // Сброс программы и устройства
      ResetDeviceProgram();
      ResetAll();
    }
    if(cmd=="stop" || cmd=="debug")
      was_cmd_debug_regime = true;
    if(cmd=="start")
      was_cmd_debug_regime = false;
  }
}

void kwc::MakeProgramStep(void)
{
  for(byte i=0;i<_kwc_NUM_DEV;i++)
    if(Program.enabled[i])
    {
      int rt = Program.ratio[i];
      if(GlobalProgCounter % rt == 0)
      {
        if(Program.tcnt[i]>=PROG_LEN) Program.tcnt[i] = 0;
        int tpos = Program.tcnt[i];
        int val = Program.data[i][tpos]*progr_scale_A;
        if(val>dev[i]->max_bound) val=dev[i]->max_bound;
        if(val<dev[i]->min_bound) val=dev[i]->min_bound;
        // Ограничения на значения для PWM-устройств
        if((i>=_kwc_NUM_DEV-_kwc_NUM_PWM_DEV) && (val>GLOBAL_PWM_LIMIT)) val = GLOBAL_PWM_LIMIT;
        dev[i]->SetGoal(val);
        Program.tcnt[i]++;
        if(i==0)
        {
          Serial.print(Program.data[i][tpos]);
          Serial.print(' ');
          Serial.print(val);
          Serial.println();
        }        
      }
    }
 }

/*
 * Отработка заданий
 */
void kwc::ExecuteProgram(void)
{
  for(byte i=0;i<_kwc_NUM_DEV;i++)
    if(Program.enabled[i])
    {
      dev[i]->Eval();
      // Важная задержка. Иначе работать не будет
      delay(20);
    }
}

void kwc::Read_and_Display_Regimes(void)
{
  static byte pred_debug_regime = 255;
  ReadSensors();
  // Флаг режима отладки
  debug_regime = !digitalRead(PIN_REGIME_SWITCH) || was_cmd_debug_regime;

  if(pred_debug_regime != debug_regime)
  {
    pred_debug_regime = debug_regime;
    if (debug_regime)
      LCDprint(0, 10, "Dbg");
    else
      LCDprint(0, 10, "Wrk");
    digitalWrite(PIN_DEBUG_REG_INDICATOR, debug_regime);
  }
  // Чтение ограничивающего резистора
  GLOBAL_PWM_LIMIT = sData[_kwc_NUM_US_SENS+0];
  GLOBAL_PWM_LIMIT = map(GLOBAL_PWM_LIMIT, 0, 1024, 0, 100);
  LCDprint(0, 14, kwc::GLOBAL_PWM_LIMIT);
}