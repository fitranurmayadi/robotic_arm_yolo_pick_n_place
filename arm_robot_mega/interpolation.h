// interpolation.h
#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <Arduino.h>

class Interpolation {
public:
    Interpolation();

    // Set target position for interpolation
    void setInterpolation(float targetX, float targetY, float targetZ, float targetE, float feedRate);

    // Update actual position during interpolation
    void updateActualPosition();

    // Check if interpolation is finished
    bool isFinished() const;

    // Get current interpolated position
    float getX() const { return currentX; }
    float getY() const { return currentY; }
    float getZ() const { return currentZ; }
    float getE() const { return currentE; }

    // Set current position directly (e.g., after homing or manual movement)
    void setCurrentPos(float x, float y, float z, float e);

private:
    float startX, startY, startZ, startE;
    float targetX, targetY, targetZ, targetE;
    float currentX, currentY, currentZ, currentE;
    float feedRate; // mm/min
    unsigned long startTime;
    float totalDistance;
    bool finished;
};

#endif
