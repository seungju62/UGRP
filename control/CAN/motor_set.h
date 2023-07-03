#ifndef motor_set_h
#define motor_set_h

#include "Arduino.h"

class motor_set
{
public:
  motor_set(int num);
  void On();

private:
  int _num;
};
#endif