/**
* \file ultrasonic.ino
* Progect:
*  Проект _______
*  Датчики Usonic 
*  
*
*  Протокол _______
*  Baudrate 9600
*  Порт - Serial
*  Chip Arduino Nano
*  \author Robofob
*  \version 0.1
*  \date 18.10.2018
*  \date Last Change: 18.10.2018
*/
//-------------------------------------------
#include "Ultrasonic.h"

#define BAUD_RATE 9600

// Ultrasonic(Trig, Echo, [timeout])
#define US_T_L 2
#define US_E_L 3
#define US_T_R 4
#define US_E_R 5
#define US_T_F 6
#define US_E_F 7
#define US_T_B 8
#define US_E_B 9

#define INFINITE 500

// Основные УЗД (рефлексы)
#define US_TIMEOUT 40000L
Ultrasonic US_L(US_T_L, US_E_L, US_TIMEOUT);
Ultrasonic US_R(US_T_R, US_E_R, US_TIMEOUT);
Ultrasonic US_F(US_T_F, US_E_F, US_TIMEOUT);
Ultrasonic US_B(US_T_B, US_E_B, US_TIMEOUT);

#define BUTTON_RED 10
#define BUTTON_GREEN 12
//#define BUTTON_LEFT 1
#define BUTTON_RIGHT 11
#define BUTTON_PRESS_JOY 13

#define JOYSTICK_1 A4
#define JOYSTICK_2 A3

#define MODE_HIGH  0
#define MODE_LOW 1

void setup() {
	Serial.begin(BAUD_RATE);

	// Ultrasonc setup
	pinMode(US_T_L, OUTPUT);
	pinMode(US_E_L, INPUT);
	pinMode(US_T_R, OUTPUT);
	pinMode(US_E_R, INPUT);
	pinMode(US_T_F, OUTPUT);
	pinMode(US_E_F, INPUT);
	pinMode(US_T_B, OUTPUT);
	pinMode(US_E_B, INPUT);

  pinMode(BUTTON_RED, INPUT);
  pinMode(BUTTON_GREEN, INPUT);
  // pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_RIGHT, INPUT);
  pinMode(BUTTON_PRESS_JOY, INPUT);

  pinMode(JOYSTICK_1, INPUT);
  pinMode(JOYSTICK_2, INPUT);

  // pullup to high
  pinMode(BUTTON_RED, INPUT_PULLUP);
  pinMode(BUTTON_GREEN, INPUT_PULLUP);
  // pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_PRESS_JOY, INPUT_PULLUP);
}

int us_l = 0;
int us_r = 0;
int us_f = 0;
int us_b = 0;

int joy_1 = 0;
int joy_2 = 0;

int button_red = MODE_LOW;
int button_green = MODE_LOW;
int button_left = MODE_LOW;
int button_right = MODE_LOW;
int button_press_joy = MODE_LOW;


void loop() {

  // switch mode if other mode is selected and the button is in corresponding state
  // red button
  if ( (digitalRead(BUTTON_RED) == LOW) && (button_red == MODE_HIGH) )  {
      button_red = MODE_LOW;
  }
  else if ( (digitalRead(BUTTON_RED) == HIGH) && (button_red == MODE_LOW) )  {
      button_red = MODE_HIGH;
  }

  // green button
  if ( (digitalRead(BUTTON_GREEN) == LOW) && (button_green == MODE_HIGH) )  {
      button_green = MODE_LOW;
  }
  else if ( (digitalRead(BUTTON_GREEN) == HIGH) && (button_green == MODE_LOW) )  {
      button_green = MODE_HIGH;
  }

  // left button
  /*
  if ( (digitalRead(BUTTON_LEFT) == LOW) && (button_left == MODE_HIGH) )  {
      button_left = MODE_LOW;
  }
  else if ( (digitalRead(BUTTON_LEFT) == HIGH) && (button_left == MODE_LOW) )  {
      button_left = MODE_HIGH;
  }
  */

  // right button
  if ( (digitalRead(BUTTON_RIGHT) == LOW) && (button_right == MODE_HIGH) )  {
      button_right = MODE_LOW;
  }
  else if ( (digitalRead(BUTTON_RIGHT) == HIGH) && (button_right == MODE_LOW) )  {
      button_right = MODE_HIGH;
  }

  // press joy button
  if ( (digitalRead(BUTTON_PRESS_JOY) == LOW) && (button_press_joy == MODE_HIGH) )  {
      button_press_joy = MODE_LOW;
  }
  else if ( (digitalRead(BUTTON_PRESS_JOY) == HIGH) && (button_press_joy == MODE_LOW) )  {
      button_press_joy = MODE_HIGH;
  }
 
	us_l = US_L.Ranging(CM);
	us_r = US_R.Ranging(CM);
	us_f = US_F.Ranging(CM);
	us_b = US_B.Ranging(CM);

	if (us_l == 0) { us_l = INFINITE; }
	if (us_r == 0) { us_r = INFINITE; }
	if (us_f == 0) { us_f = INFINITE; }
	if (us_b == 0) { us_b = INFINITE; }

  joy_1 = analogRead(JOYSTICK_1);
  joy_2 = analogRead(JOYSTICK_2);

  // rangers
	Serial.print(us_l);
	Serial.print(' ');
	Serial.print(us_r);
	Serial.print(' ');
	Serial.print(us_f);
	Serial.print(' ');
	Serial.print(us_b);

  // joystick
  Serial.print(' ');
  Serial.print(joy_2);
  Serial.print(' ');
  Serial.print(joy_1);

  // joystick buttons
  Serial.print(' ');
  Serial.print(button_red);
  Serial.print(' ');
  Serial.print(button_green);
  Serial.print(' ');
  Serial.print(button_left);
  Serial.print(' ');
  Serial.print(button_right);
  Serial.print(' ');
  Serial.print(button_press_joy);

  Serial.println();
  
  

}
