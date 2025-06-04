// RampsStepper.h
#ifndef RAMPS_STEPPER_H
#define RAMPS_STEPPER_H

#include <Arduino.h>

class RampsStepper {
public:
  // Konstruktor baru dengan pin limit, arah homing, dan arah terbalik
  RampsStepper(int stepPin, int dirPin, int enablePin, int limitPin, bool dirHighToHome, bool reverseDirection);
  
  void enable(bool flag);
  void disable();
  void stepRelative(long steps); // Menggunakan long untuk langkah
  void stepToPosition(long steps); // Menggunakan long untuk langkah
  void stepToPositionRad(float rad);

  // Set rasio gearbox dan steps/rev (untuk konversi rad -> steps)
  void setReductionRatio(float gearRatio, long stepsPerRevRaw); // Menggunakan long untuk raw steps
  void setStepDelay(unsigned int delayUs); // Menggunakan unsigned int untuk delay
  
  bool isMoving() const; 
  void setPosition(long steps); // Menggunakan long untuk posisi
  long getPosition() const; // Menggunakan long untuk posisi
  bool isOnTarget() const;

  float getRadToStepFactor() const; // Getter untuk faktor konversi
  float getStepToRadFactor() const; // Getter untuk faktor konversi

  void update(); // Fungsi update utama, dipanggil di loop

  // Getter functions
  int getStepPin() const { return stepPin; }
  int getDirPin() const { return dirPin; }
  int getEnablePin() const { return enablePin; } 
  int getLimitPin() const { return limitPin; }
  bool getDirHighToHome() const { return dirHighToHome; }
  bool getReverseDirection() const { return reverseDirection; } // New getter for reverseDirection
  bool isLimitActive() const; // Fungsi untuk memeriksa status limit switch

private:
  int stepPin, dirPin, enablePin, limitPin;
  bool dirHighToHome;
  bool reverseDirection; // New member variable
  long currentStep, targetStep; 
  bool moving; // Status apakah motor sedang bergerak
  unsigned int stepDelay; // Variabel anggota untuk menyimpan nilai step delay
  unsigned long lastStepTime; // Waktu terakhir langkah diambil

  float reductionRatio;
  long stepsPerRevolutionRaw; // Langkah mentah per putaran motor (misal 200 * 16 microsteps)
  
  // Fungsi helper untuk melakukan satu langkah, sekarang diimplementasikan di RampsStepper.cpp
  // dan akan menggunakan logika reverseDirection.
  // void stepMotor(int pin, bool direction); // Tidak lagi diperlukan sebagai prototipe di sini
};

#endif
