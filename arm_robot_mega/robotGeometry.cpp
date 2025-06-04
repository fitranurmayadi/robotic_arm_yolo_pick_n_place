// robotGeometry.cpp
#include <Arduino.h>
#include "robotGeometry.h"
#include <math.h>

// Panjang link robot dalam milimeter (mm):
// L1: Tinggi vertikal sendi Bahu dari permukaan dasar robot (Base height to Shoulder)
// L2: Panjang link Bahu ke Siku (Shoulder to Elbow)
// L3: Panjang link Siku ke Pergelangan/Titik Referensi End-Effector (Elbow to Wrist)
// PENTING: Nilai-nilai ini HARUS diukur dengan sangat akurat dari robot fisik.
// Kesalahan pengukuran sekecil apapun akan menyebabkan ketidakakuratan dalam Inverse Kinematics.
// Perhatikan: Variabel ini dideklarasikan di header RobotGeometry.h sebagai static const.
// Kita hanya perlu memastikan nilainya konsisten di sini jika didefinisikan ulang,
// atau menghapusnya jika sudah didefinisikan di header.
// Untuk tujuan refactoring, kita akan menggunakannya dari header.

RobotGeometry::RobotGeometry() {
  // Inisialisasi semua variabel anggota di konstruktor
  x_target = y_target = z_target = 0.0;
  base_rad = sh_rad = el_rad = 0.0;
  fk_x = fk_y = fk_z = 0.0; 
  _useElbowDownSolution = false; // Inisialisasi flag solusi IK ke Elbow Up

  // Inisialisasi offset kinematik dan Kartesian
  kinematicBaseZeroOffsetRad = 0.0;
  kinematicShoulderZeroOffsetRad = 0.0;
  kinematicElbowZeroOffsetRad = 0.0;
  cartesianOffsetX = 0.0;
  cartesianOffsetY = 0.0;
  cartesianOffsetZ = 0.0;
}

// Mengatur target posisi End-Effector (dalam mm) dan menghitung sudut sendi (Inverse Kinematics)
void RobotGeometry::setPositionCartesianOffset(float x_mm, float y_mm, float z_mm) {
  x_target = x_mm;
  y_target = y_mm;
  z_target = z_mm;
  calculateIK();
}

// Implementasi fungsi calculateIK (Inverse Kinematics)
void RobotGeometry::calculateIK() {
  // === Inverse Kinematics (IK) ===
  // Tujuan: Menghitung sudut sendi (base_rad, sh_rad, el_rad) untuk mencapai target X, Y, Z.
  // Karena End-Effector memiliki offset tetap dari Wrist (titik akhir L3), kita perlu menghitung
  // posisi target Wrist terlebih dahulu.

  // 1. Kurangkan offset Kartesian dari target sebelum perhitungan IK
  float x_ik = x_target - cartesianOffsetX;
  float y_ik = y_target - cartesianOffsetY;
  float z_ik = z_target - cartesianOffsetZ;

  // Hitung posisi Wrist Center (WC) yang diperlukan untuk mencapai target EE
  // Radial distance dari WC ke origin di bidang XY
  float ree_target = sqrt(x_ik * x_ik + y_ik * y_ik);
  float r_wc_target = ree_target - EE_FORWARD_OFFSET_MM;
  if (r_wc_target < 0) r_wc_target = 0; // Pastikan tidak negatif

  // Tinggi relatif WC ke Shoulder Joint
  // Karena EE_DOWN_OFFSET_MM adalah offset ke bawah dari Wrist,
  // untuk mendapatkan tinggi Wrist, kita harus MENAMBAHKAN offset ini ke target Z EE.
  float z_wc_target_rel_sh = (z_ik - L1) + EE_DOWN_OFFSET_MM; // L1 adalah tinggi Base ke Shoulder

  // 2. Hitung d (jarak lurus dari Shoulder Joint ke Wrist Center)
  float d = sqrt(r_wc_target * r_wc_target + z_wc_target_rel_sh * z_wc_target_rel_sh);

  // Batasan workspace untuk 'd':
  // Pastikan d tidak lebih kecil dari selisih absolut L2 dan L3, atau lebih besar dari jumlah L2 dan L3
  if (d < fabs(L2 - L3)) d = fabs(L2 - L3);
  if (d > (L2 + L3))      d = L2 + L3;
  if (d < 0.001) d = 0.001; // Hindari pembagian dengan nol

  // 3. Hitung sudut internal elbow (phi): φ
  // Menggunakan hukum cosinus pada segitiga yang dibentuk oleh L2, L3, dan d
  float cos_phi = (L2 * L2 + L3 * L3 - d * d) / (2.0 * L2 * L3);
  cos_phi = constrain(cos_phi, -1.0f, 1.0f); // Batasi nilai cos_phi antara -1 dan 1
  float phi = acos(cos_phi);

  // 4. Hitung sudut shoulder (alpha dan beta):
  // Alpha adalah sudut dari sumbu horizontal ke garis d.
  // Beta adalah sudut dari L2 ke d.
  float alpha = atan2(z_wc_target_rel_sh, r_wc_target);
  float cos_beta = (L2 * L2 + d * d - L3 * L3) / (2.0 * L2 * d);
  cos_beta = constrain(cos_beta, -1.0f, 1.0f);
  float beta = acos(cos_beta);

  if (_useElbowDownSolution) {
    // Solusi "Elbow Down" (Siku ke Bawah / Lefty)
    sh_rad = alpha - beta;
    el_rad = phi - M_PI; // Sudut elbow relatif terhadap lengan shoulder
  } else {
    // Solusi "Elbow Up" (Siku ke Atas / Righty) - Ini adalah default
    sh_rad = alpha + beta;
    el_rad = M_PI - phi; // Sudut elbow relatif terhadap lengan shoulder
  }

  // 5. Hitung sudut base (theta1):
  // Sudut Base dihitung dari posisi X dan Y target di bidang horizontal.
  base_rad = atan2(y_ik, x_ik);

  // Kurangkan offset kinematik dari sudut yang dihitung IK
  // agar sudut yang dikembalikan sesuai dengan '0 langkah' stepper yang dikalibrasi.
  base_rad -= kinematicBaseZeroOffsetRad;
  sh_rad -= kinematicShoulderZeroOffsetRad;
  el_rad -= kinematicElbowZeroOffsetRad;
}

// Mengatur apakah akan menggunakan solusi Inverse Kinematics "Elbow Down"
void RobotGeometry::setUseElbowDownSolution(bool useDown) {
  _useElbowDownSolution = useDown;
}

// Mengatur offset Kartesian global untuk sistem koordinat robot
void RobotGeometry::setCartesianOffset(float x, float y, float z) {
    cartesianOffsetX = x;
    cartesianOffsetY = y;
    cartesianOffsetZ = z;
    Serial.print("DEBUG: Offset Kartesian diatur ke [");
    Serial.print(cartesianOffsetX); Serial.print(", ");
    Serial.print(cartesianOffsetY); Serial.print(", ");
    Serial.print(cartesianOffsetZ); Serial.println("]");
}

// Mengatur offset untuk posisi nol kinematik setiap sendi
void RobotGeometry::setKinematicZeroOffsets(float baseOffsetRad, float shoulderOffsetRad, float elbowOffsetRad) {
    kinematicBaseZeroOffsetRad = baseOffsetRad;
    kinematicShoulderZeroOffsetRad = shoulderOffsetRad;
    kinematicElbowZeroOffsetRad = elbowOffsetRad;
    Serial.print("DEBUG: Offset nol kinematik diatur ke [Base="); Serial.print(degrees(baseOffsetRad), 2);
    Serial.print("deg, Shoulder="); Serial.print(degrees(shoulderOffsetRad), 2);
    Serial.print("deg, Elbow="); Serial.print(degrees(elbowOffsetRad), 2); Serial.println("deg]");
}

// Getter untuk sudut Base (dari IK)
float RobotGeometry::getBaseRad() const {
  return base_rad;
}

// Getter untuk sudut Shoulder (dari IK)
float RobotGeometry::getShoulderRad() const {
  return sh_rad;
}

// Getter untuk sudut Elbow (dari IK)
float RobotGeometry::getElbowRad() const {
  return el_rad;
}

// === Forward Kinematics (FK) ===
// Tujuan: Menghitung posisi Kartesian (X, Y, Z) dari sudut-sudut sendi (base_rad_in, shoulder_rad_in, elbow_rad_in).
void RobotGeometry::calculateFK(float base_rad_in, float shoulder_rad_in, float elbow_rad_in) {
  // Tambahkan offset kinematik ke sudut yang diterima (sudut dari stepper)
  // agar sesuai dengan sistem koordinat kinematik model.
  float adjustedBaseRad = base_rad_in + kinematicBaseZeroOffsetRad;
  float adjustedShoulderRad = shoulder_rad_in + kinematicShoulderZeroOffsetRad;
  float adjustedElbowRad = elbow_rad_in + kinematicElbowZeroOffsetRad;

  // Sudut relatif Elbow terhadap Shoulder (phi dari IK)
  // Ini adalah sudut yang dibentuk oleh L2 dan L3
  float phi_fk;
  if (_useElbowDownSolution) {
      phi_fk = M_PI + adjustedElbowRad; 
  } else {
      phi_fk = M_PI - adjustedElbowRad; 
  }

  // 1. Hitung posisi Wrist Center (WC) relatif terhadap Shoulder Joint
  // Menggunakan L2 dan L3
  float r_wc_rel_sh = L2 * cos(adjustedShoulderRad) + L3 * cos(adjustedShoulderRad + phi_fk);
  float z_wc_rel_sh = L2 * sin(adjustedShoulderRad) + L3 * sin(adjustedShoulderRad + phi_fk);

  // 2. Hitung posisi Wrist Center (WC) dalam koordinat dunia (relatif terhadap Base)
  // Menambahkan tinggi Base ke Shoulder (L1) ke komponen Z
  float wc_x_world_radial = r_wc_rel_sh;
  float wc_z_world = z_wc_rel_sh + L1; // L1 adalah tinggi Base ke Shoulder

  // 3. Tambahkan offset End-Effector (EE) dari Wrist Center (WC)
  // EE_FORWARD_OFFSET_MM ditambahkan ke komponen radial
  // EE_DOWN_OFFSET_MM dikurangi dari komponen Z (karena ke bawah)
  float ee_x_world_radial = wc_x_world_radial + EE_FORWARD_OFFSET_MM;
  float ee_z_world = wc_z_world - EE_DOWN_OFFSET_MM; 

  // 4. Rotasi berdasarkan sudut Base (adjustedBaseRad) untuk mendapatkan final X, Y, Z
  // Kemudian tambahkan offset Kartesian global
  fk_x = ee_x_world_radial * cos(adjustedBaseRad) + cartesianOffsetX; 
  fk_y = ee_x_world_radial * sin(adjustedBaseRad) + cartesianOffsetY; 
  fk_z = ee_z_world + cartesianOffsetZ; 
}

// Getter untuk posisi X (dari FK)
float RobotGeometry::getFKX() const { return fk_x; }
// Getter untuk posisi Y (dari FK)
float RobotGeometry::getFKY() const { return fk_y; }
// Getter untuk posisi Z (dari FK)
float RobotGeometry::getFKZ() const { return fk_z; }
