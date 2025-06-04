// fanControl.h
#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H
#include <Arduino.h>

class FanControl {
public:
  FanControl(int pin);
  void enable(bool flag);    // HIGH = ON, LOW = OFF
  void update();             // jika butuh PWM, isi logic di sini
private:
  int pin;
};

#endif