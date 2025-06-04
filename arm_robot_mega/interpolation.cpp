// interpolation.cpp
#include "interpolation.h"
#include <math.h> // Untuk sqrt, pow

Interpolation::Interpolation() {
    startX = startY = startZ = startE = 0.0;
    targetX = targetY = targetZ = targetE = 0.0;
    currentX = currentY = currentZ = currentE = 0.0;
    feedRate = 0.0;
    startTime = 0;
    totalDistance = 0.0;
    finished = true; // Awalnya dianggap selesai
}

// Mengatur posisi target untuk interpolasi
void Interpolation::setInterpolation(float tx, float ty, float tz, float te, float fr) {
    startX = currentX;
    startY = currentY;
    startZ = currentZ;
    startE = currentE;

    targetX = tx;
    targetY = ty;
    targetZ = tz;
    targetE = te;

    feedRate = fr; // mm/min

    // Hitung jarak total yang harus ditempuh
    float dx = targetX - startX;
    float dy = targetY - startY;
    float dz = targetZ - startZ;
    float de = targetE - startE; // Untuk slider

    totalDistance = sqrt(dx*dx + dy*dy + dz*dz + de*de); // Jarak Euclidean di ruang 4D (XYZ + E)

    startTime = millis();
    finished = false;

    // Jika jaraknya sangat kecil, anggap sudah selesai
    if (totalDistance < 0.001) { // Threshold kecil untuk menghindari pembagian nol atau gerakan mikro
        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
        currentE = targetE;
        finished = true;
    }
}

// Memperbarui posisi aktual selama interpolasi
void Interpolation::updateActualPosition() {
    if (finished) return;

    unsigned long elapsedTime = millis() - startTime; // Waktu yang telah berlalu dalam ms
    float distanceMoved = (feedRate / 60000.0) * elapsedTime; // Jarak yang seharusnya ditempuh dalam mm (mm/ms)

    if (distanceMoved >= totalDistance) {
        // Jika sudah mencapai atau melewati target, set posisi ke target akhir
        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
        currentE = targetE;
        finished = true;
    } else {
        // Hitung posisi saat ini berdasarkan proporsi jarak yang ditempuh
        float ratio = distanceMoved / totalDistance;
        currentX = startX + (targetX - startX) * ratio;
        currentY = startY + (targetY - startY) * ratio;
        currentZ = startZ + (targetZ - startZ) * ratio;
        currentE = startE + (targetE - startE) * ratio;
    }
}

// Memeriksa apakah interpolasi telah selesai
bool Interpolation::isFinished() const {
    return finished;
}

// Mengatur posisi saat ini secara langsung (misalnya setelah homing)
void Interpolation::setCurrentPos(float x, float y, float z, float e) {
    currentX = x;
    currentY = y;
    currentZ = z;
    currentE = e;
    finished = true; // Setelah diatur, anggap tidak ada gerakan yang tertunda
}
