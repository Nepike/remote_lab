#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

class Motor
{
  public:
    Motor(int pinM, int pinE);
    void GoFwd(int Speed);
    void GoBack(int Speed);
    void Stop();
    void Go(int Speed);
  private:
    int _M;
    int _E;
    int _speed;
};

#endif