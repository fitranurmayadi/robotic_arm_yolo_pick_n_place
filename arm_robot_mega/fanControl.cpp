// fanControl.cpp
#include "fanControl.h"

FanControl::FanControl(int aPin) {
  pin = aPin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void FanControl::enable(bool flag) {
  digitalWrite(pin, flag ? HIGH : LOW);
}

void FanControl::update() {
  // Jika Anda tidak menggunakan PWM, kosongkan saja.
  // Jika menggunakan PWM, adjust nilai analogWrite(pin, nilai);
}