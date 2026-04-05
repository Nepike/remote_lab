#include "Motor.h"

Motor::Motor(int pinM, int pinE)
{
  _M = pinM;
  _E = pinE;
  pinMode(_M, OUTPUT);
}

void Motor::GoFwd(int Speed)
{
  if( Speed <= 255 )
    _speed = Speed;
  else
    _speed = 255;
  digitalWrite(_M, HIGH);
  analogWrite(_E, _speed);
}

void Motor::GoBack(int Speed)
{
  if( Speed < 255 )
    _speed = Speed;
  else
    _speed = 255;
  digitalWrite(_M, LOW);
  analogWrite(_E, _speed);
}

void Motor::Stop()
{
  analogWrite(_E,0);
}

void Motor::Go(int Speed)
{
  _speed = Speed;
  if(_speed < 0)
  {
    digitalWrite(_M, LOW);
    analogWrite(_E, -_speed);
  }
  else
    if(_speed > 0)
    {
      digitalWrite(_M, HIGH);
      analogWrite(_E, _speed);
    }
    else
      analogWrite(_E, 0);
}

