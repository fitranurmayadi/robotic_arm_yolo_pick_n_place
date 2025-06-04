// RampsStepper.cpp
#include "RampsStepper.h"
#include <Arduino.h>
#include <math.h> // Digunakan untuk M_PI jika diperlukan dalam perhitungan

// Konstruktor RampsStepper
// stepPin, dirPin, enablePin: Pin untuk kontrol motor stepper
// limitPin: Pin untuk limit switch
// dirHighToHome: TRUE jika arah HIGH (CW) pada dirPin menggerakkan motor menuju limit switch
// reverseDirection: TRUE jika arah fisik motor terbalik dari arah logis yang diinginkan
RampsStepper::RampsStepper(int aStepPin, int aDirPin, int aEnablePin, int aLimitPin, bool aDirHighToHome, bool aReverseDirection)
    : stepPin(aStepPin), dirPin(aDirPin), enablePin(aEnablePin), limitPin(aLimitPin), dirHighToHome(aDirHighToHome), reverseDirection(aReverseDirection) {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(limitPin, INPUT_PULLUP); // Limit switch biasanya menggunakan INPUT_PULLUP

    currentStep = 0;
    targetStep = 0;
    moving = false;
    stepDelay = 100; // Default delay
    lastStepTime = 0;

    reductionRatio = 1.0; // Default tanpa pengurangan gigi
    stepsPerRevolutionRaw = 200 * 16; // Default untuk motor 1.8 derajat dengan 1/16 microstepping
}

// Mengaktifkan driver motor stepper
void RampsStepper::enable(bool flag) {
    digitalWrite(enablePin, flag ? LOW : HIGH); // LOW = ENABLE, HIGH = DISABLE (umum untuk driver A4988/DRV8825)
}

// Menonaktifkan driver motor stepper
void RampsStepper::disable() {
    digitalWrite(enablePin, HIGH); // HIGH = DISABLE
}

// Menggerakkan motor sejumlah langkah relatif dari posisi saat ini
void RampsStepper::stepRelative(long steps) {
    targetStep = currentStep + steps;
    moving = true;
}

// Menggerakkan motor ke posisi langkah absolut
void RampsStepper::stepToPosition(long steps) {
    targetStep = steps;
    moving = true;
}

// Menggerakkan motor ke posisi sudut absolut (radian)
void RampsStepper::stepToPositionRad(float rad) {
    // Konversi radian ke langkah menggunakan faktor konversi
    targetStep = (long)(rad * getRadToStepFactor());
    moving = true;
}

// Mengatur rasio pengurangan gigi dan langkah per putaran motor mentah
void RampsStepper::setReductionRatio(float gearRatio, long stepsPerRevRaw) {
    reductionRatio = gearRatio;
    stepsPerRevolutionRaw = stepsPerRevRaw;
}

// Mengatur delay antar langkah (dalam mikrodetik)
void RampsStepper::setStepDelay(unsigned int delayUs) {
    stepDelay = delayUs;
}

// Memeriksa apakah motor sedang bergerak
bool RampsStepper::isMoving() const {
    return moving;
}

// Mengatur posisi langkah internal motor
void RampsStepper::setPosition(long steps) {
    currentStep = steps;
    targetStep = steps;
    moving = false;
}

// Mendapatkan posisi langkah internal motor saat ini
long RampsStepper::getPosition() const {
    return currentStep;
}

// Memeriksa apakah motor telah mencapai target posisi
bool RampsStepper::isOnTarget() const {
    return currentStep == targetStep;
}

// Mendapatkan faktor konversi dari radian ke langkah
float RampsStepper::getRadToStepFactor() const {
    // (stepsPerRevolutionRaw * reductionRatio) / (2 * PI)
    // Jika reductionRatio negatif, ini akan membalikkan arah langkah.
    return (float)stepsPerRevolutionRaw * reductionRatio / (2.0 * M_PI);
}

// Mendapatkan faktor konversi dari langkah ke radian
float RampsStepper::getStepToRadFactor() const {
    return (2.0 * M_PI) / ((float)stepsPerRevolutionRaw * reductionRatio);
}

// Memeriksa status limit switch
bool RampsStepper::isLimitActive() const {
    return digitalRead(limitPin) == LOW; // Asumsi limit switch aktif LOW
}

// Fungsi update utama, dipanggil di loop Arduino
void RampsStepper::update() {
    if (!moving) return;

    if (currentStep != targetStep) {
        unsigned long currentTime = micros();
        if (currentTime - lastStepTime >= stepDelay) {
            // Tentukan arah langkah yang diinginkan (apakah target lebih besar atau lebih kecil dari posisi saat ini)
            bool steppingTowardsPositiveSteps = (targetStep > currentStep);

            // Tentukan keadaan pin arah mentah berdasarkan `dirHighToHome`
            // `dirHighToHome` TRUE berarti HIGH pada dirPin menggerakkan motor ke arah limit switch.
            // Asumsi: arah limit switch adalah arah "negatif" untuk homing.
            // Jadi, jika `dirHighToHome` TRUE, maka HIGH adalah arah negatif. LOW adalah arah positif.
            // Jika `dirHighToHome` FALSE, maka LOW adalah arah negatif. HIGH adalah arah positif.
            
            // Keadaan pin arah fisik yang akan diatur, TANPA mempertimbangkan `reverseDirection` dulu.
            // Jika kita melangkah ke arah positif langkah:
            //   Jika `dirHighToHome` TRUE (HIGH = negatif), maka kita butuh LOW untuk positif.
            //   Jika `dirHighToHome` FALSE (LOW = negatif), maka kita butuh HIGH untuk positif.
            // Jadi, `rawDirPinState` untuk langkah positif adalah `!dirHighToHome`.
            bool rawDirPinState;
            if (steppingTowardsPositiveSteps) {
                rawDirPinState = !dirHighToHome; 
            } else { // steppingTowardsNegativeSteps
                rawDirPinState = dirHighToHome;
            }
            
            // Terapkan `reverseDirection` untuk mendapatkan keadaan pin arah fisik yang sebenarnya
            bool actualDirPinState = reverseDirection ? !rawDirPinState : rawDirPinState;

            // --- Pengecekan Limit Switch ---
            // Hanya hentikan pergerakan jika limit switch aktif DAN kita mencoba bergerak menuju limit.
            // Bergerak menuju limit berarti `actualDirPinState` sama dengan `dirHighToHome`.
            if (isLimitActive()) {
                if (actualDirPinState == dirHighToHome) {
                    Serial.print("WARNING: Limit switch ");
                    Serial.print(limitPin);
                    Serial.print(" active for stepper ");
                    Serial.print(stepPin);
                    Serial.println(". Stopping movement.");
                    moving = false;
                    targetStep = currentStep; // Atur target ke posisi saat ini untuk berhenti
                    return; // Keluar dari siklus update
                }
            }

            // Atur pin arah
            digitalWrite(dirPin, actualDirPinState); 

            // Lakukan satu langkah
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(stepDelay / 2); // Durasi pulsa HIGH
            digitalWrite(stepPin, LOW);
            delayMicroseconds(stepDelay / 2); // Durasi pulsa LOW

            // Perbarui posisi saat ini
            if (targetStep > currentStep) {
                currentStep++;
            } else {
                currentStep--;
            }
            lastStepTime = currentTime;
        }
    } else {
        moving = false; // Berhenti bergerak jika sudah di target
    }
}
