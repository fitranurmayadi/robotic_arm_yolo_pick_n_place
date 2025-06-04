#ifndef PINOUT_H
#define PINOUT_H

// Definisi Pin untuk RAMPS 1.4
// Asumsi: Ini adalah pin standar pada board RAMPS 1.4
// dan motor fisik Anda terhubung ke slot-slot ini.

// --- RAMPS X-Axis Pins ---
// Ini secara fisik terhubung ke motor ELBOW Anda
#define X_STEP_PIN          54
#define X_DIR_PIN           55
#define X_ENABLE_PIN        38
#define X_MIN_PIN            3 // Limit switch untuk X-axis (Elbow)
#define X_MAX_PIN            2 // Jika digunakan, limit switch di ujung lain

// --- RAMPS Y-Axis Pins ---
// Ini secara fisik terhubung ke motor SHOULDER Anda
#define Y_STEP_PIN          60
#define Y_DIR_PIN           61
#define Y_ENABLE_PIN        56
#define Y_MIN_PIN           14 // Limit switch untuk Y-axis (Shoulder)
#define Y_MAX_PIN           15 // Jika digunakan, limit switch di ujung lain

// --- RAMPS Z-Axis Pins ---
// Ini secara fisik terhubung ke motor BASE (ROTATE) Anda
#define Z_STEP_PIN          46
#define Z_DIR_PIN           48
#define Z_ENABLE_PIN        62
#define Z_MIN_PIN           18 // Limit switch untuk Z-axis (Base)
#define Z_MAX_PIN           19 // Jika digunakan, limit switch di ujung lain

// --- Alias Pin untuk Sumbu Robot Arm ---
// Ini adalah penamaan yang lebih fungsional untuk robot arm Anda
// dan memetakan ke pin RAMPS yang sesuai.

// Base (Rotate) Axis - Menggunakan pin RAMPS Z-Axis
#define ROTATE_STEP_PIN     Z_STEP_PIN
#define ROTATE_DIR_PIN      Z_DIR_PIN
#define ROTATE_ENABLE_PIN   Z_ENABLE_PIN
#define ROTATE_LIMIT_PIN    Z_MIN_PIN // Menggunakan Z_MIN_PIN sebagai limit home

// Shoulder Axis - Menggunakan pin RAMPS Y-Axis
#define SHOULDER_STEP_PIN   Y_STEP_PIN
#define SHOULDER_DIR_PIN    Y_DIR_PIN
#define SHOULDER_ENABLE_PIN Y_ENABLE_PIN
#define SHOULDER_LIMIT_PIN  Y_MIN_PIN // Menggunakan Y_MIN_PIN sebagai limit home

// Elbow Axis - Menggunakan pin RAMPS X-Axis
#define ELBOW_STEP_PIN      X_STEP_PIN
#define ELBOW_DIR_PIN       X_DIR_PIN
#define ELBOW_ENABLE_PIN    X_ENABLE_PIN
#define ELBOW_LIMIT_PIN     X_MIN_PIN // Menggunakan X_MIN_PIN sebagai limit home

// Slider atau End-Effector (E-axis) - Menggunakan pin AUX-2 atau pin lain yang tersedia
#define SLIDER_STEP_PIN     36
#define SLIDER_DIR_PIN      34
#define SLIDER_ENABLE_PIN   30
#define SLIDER_LIMIT_PIN    67 // Pin limit untuk slider, sesuaikan jika berbeda

// Gripper (stepper bipolar via AUX-1)
#define GRIPPER_PIN0        40
#define GRIPPER_PIN1        59
#define GRIPPER_PIN2        63
#define GRIPPER_PIN3        64

// Suction & Fan
#define SUCTION_PIN         10
#define FAN_PIN             9

// LED Indikator
#define LED_PIN             13

#endif
