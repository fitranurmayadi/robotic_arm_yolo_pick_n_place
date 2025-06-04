// robotGeometry.h
#ifndef ROBOT_GEOMETRY_H
#define ROBOT_GEOMETRY_H

#include <math.h> // Diperlukan untuk M_PI jika digunakan di header

class RobotGeometry {
public:
  RobotGeometry();
  // Masukkan target posisi EE (dalam mm), hitung θ1, θ2, θ3 (Inverse Kinematics)
  void setPositionCartesianOffset(float x_mm, float y_mm, float z_mm); // Mengubah return type menjadi void
  float getBaseRad()    const; // θ1
  float getShoulderRad()const; // θ2
  float getElbowRad()   const; // θ3

  // Hitung posisi Kartesian (X, Y, Z) dari sudut-sudut sendi (Forward Kinematics)
  void calculateFK(float base_rad_in, float shoulder_rad_in, float elbow_rad_in);
  float getFKX() const;
  float getFKY() const;
  float getFKZ() const;

  // Fungsi untuk memilih solusi IK (Elbow Up/Down)
  void setUseElbowDownSolution(bool useDown);

  // Set offset Kartesian untuk sistem koordinat robot
  void setCartesianOffset(float x, float y, float z);

  // Set offset untuk posisi nol kinematik setiap sendi
  void setKinematicZeroOffsets(float baseOffsetRad, float shoulderOffsetRad, float elbowOffsetRad);

  // Getter untuk offset nol kinematik (BARU)
  float getKinematicBaseZeroOffsetRad() const { return kinematicBaseZeroOffsetRad; }
  float getKinematicShoulderZeroOffsetRad() const { return kinematicShoulderZeroOffsetRad; }
  float getKinematicElbowZeroOffsetRad() const { return kinematicElbowZeroOffsetRad; }

private:
  // Variabel anggota untuk menyimpan target posisi Kartesian
  float x_target, y_target, z_target;
  // Variabel anggota untuk menyimpan sudut hasil IK
  float base_rad, sh_rad, el_rad; // Sudut-sudut hasil IK
  // Variabel anggota untuk menyimpan hasil FK
  float fk_x, fk_y, fk_z; 

  // Panjang link robot dalam milimeter (mm):
  // L1: Tinggi vertikal sendi Bahu dari permukaan dasar robot (Base height to Shoulder)
  // L2: Panjang link Bahu ke Siku (Shoulder to Elbow)
  // L3: Panjang link Siku ke Pergelangan/Titik Referensi End-Effector (Elbow to Wrist)
  // PENTING: Nilai-nilai ini HARUS diukur dengan sangat akurat dari robot fisik.
  // Kesalahan pengukuran sekecil apapun akan menyebabkan ketidakakuratan dalam Inverse Kinematics.
  static const float L1 = 160.0;   // Tinggi Base ke Shoulder (sebelumnya h_sh)
  static const float L2 = 130.0;   // Shoulder ke Elbow (sebelumnya l1)
  static const float L3 = 160.0;   // Elbow ke Wrist (sebelumnya l2)

  // Offset End-Effector dari titik akhir L3 (Wrist)
  // End-effector selalu mengarah horizontal, 5cm ke depan dan 5cm ke bawah dari Wrist.
  static const float EE_FORWARD_OFFSET_MM = 50.0; // Offset horizontal ke depan (5cm)
  static const float EE_DOWN_OFFSET_MM = 50.0;    // Offset vertikal ke bawah (5cm)

  // Offset sudut untuk mengkalibrasi posisi fisik '0 langkah' motor ke '0' kinematik model
  float kinematicBaseZeroOffsetRad;
  float kinematicShoulderZeroOffsetRad;
  float kinematicElbowZeroOffsetRad;

  float cartesianOffsetX, cartesianOffsetY, cartesianOffsetZ; // Offset Kartesian baru

  void calculateIK(); // Deklarasi fungsi private

  bool _useElbowDownSolution; // Default ke Elbow Up, inisialisasi di konstruktor
};

#endif
