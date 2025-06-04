// arduino_robot_code.ino
#include <Arduino.h>
#include <Stepper.h>
#include "pinout.h"
#include "robotGeometry.h" 
#include "interpolation.h"
#include "fanControl.h" 
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include <math.h> 

// === GLOBAL OBJECTS ===
Stepper gripperStepper(2400, GRIPPER_PIN0, GRIPPER_PIN1, GRIPPER_PIN2, GRIPPER_PIN3);
// RampsStepper(stepPin, dirPin, enablePin, limitPin, dirHighToHome, reverseDirection)
// dirHighToHome: TRUE jika HIGH (CW) pada dirPin menggerakkan motor menuju limit switch.
// reverseDirection: TRUE jika arah fisik motor terbalik dari arah logis yang diinginkan (misalnya, CW motor memutar berlawanan arah dengan definisi positif).
// PENTING: Konfigurasi ini adalah yang BENAR dan TIDAK BOLEH DIUBAH.
RampsStepper stepperBase(ROTATE_STEP_PIN, ROTATE_DIR_PIN, ROTATE_ENABLE_PIN, ROTATE_LIMIT_PIN, false, false); // Base (RAMPS Z-Axis)
RampsStepper stepperShoulder(SHOULDER_STEP_PIN, SHOULDER_DIR_PIN, SHOULDER_ENABLE_PIN, SHOULDER_LIMIT_PIN, true, false); // Shoulder (RAMPS Y-Axis)
RampsStepper stepperElbow(ELBOW_STEP_PIN, ELBOW_DIR_PIN, ELBOW_ENABLE_PIN, ELBOW_LIMIT_PIN, false, false); // Elbow (RAMPS X-Axis)
RampsStepper stepperSlider(SLIDER_STEP_PIN, SLIDER_DIR_PIN, SLIDER_ENABLE_PIN, SLIDER_LIMIT_PIN, true, false); // Slider
FanControl fan(FAN_PIN); 
RobotGeometry geom; // Objek kinematika
Interpolation interpolator; // Objek interpolasi
Queue<Cmd> queue(15); // Antrian perintah G-code (M-code dan G28)
Command command; // Parser perintah G-code

// Variabel global untuk step delay (satu sumber kebenatan)
static const int GLOBAL_STEP_DELAY = 100; // Anda bisa ubah ini ke 500 jika ingin lebih lambat untuk testing

static float radPerMmSlider;
// Global constants for slider mechanics
const float pitch_mm_per_rev = 20.0; 
const float microstep_per_rev = 3200.0; 

// Definisikan posisi home End-Effector yang diinginkan dalam milimeter
// Ini adalah posisi (X, Y, Z) End-Effector ketika robot dianggap berada di "home"
// Menggunakan nilai yang Anda inginkan: X 0 Y210 Z235
const float ROBOT_HOME_X = 0.0; 
const float ROBOT_HOME_Y = 210.0;  
const float ROBOT_HOME_Z = 235.0; 
const float ROBOT_HOME_E = 0.0;  // Slider di posisi 0 mm

// === FUNCTION DECLARATIONS (Prototypes) ===
void homingAll();
void homeAxis(RampsStepper& stepper); 
void stepMotor(int pin); // Fungsi helper untuk satu langkah, digunakan internal
void executeCommand(const Cmd &cmd); // Diubah: tidak lagi menerima posisi Kartesian
bool handleDebugCommands(const String &line); // Diubah nama dan fungsionalitas
void parseAndMoveJoint(const String &line); // Pertahankan: untuk kontrol sendi langsung
void waitForMovement(long timeout_ms = 120000); 

// Prototype for smarter back-off function
void backOffUntilLimitReleased(RampsStepper& stepper, int maxSteps, int debounceDelayMs);

void setup() {
  Serial.begin(115200);

  pinMode(ROTATE_ENABLE_PIN, OUTPUT);
  pinMode(SHOULDER_ENABLE_PIN, OUTPUT);
  pinMode(ELBOW_ENABLE_PIN, OUTPUT);
  pinMode(SLIDER_ENABLE_PIN, OUTPUT);
  digitalWrite(ROTATE_ENABLE_PIN, HIGH);   
  digitalWrite(SHOULDER_ENABLE_PIN, HIGH);
  digitalWrite(ELBOW_ENABLE_PIN, HIGH);
  digitalWrite(SLIDER_ENABLE_PIN, HIGH);

  // Set step delay untuk setiap objek stepper
  stepperBase.setStepDelay(GLOBAL_STEP_DELAY);
  stepperShoulder.setStepDelay(GLOBAL_STEP_DELAY);
  stepperElbow.setStepDelay(GLOBAL_STEP_DELAY);
  stepperSlider.setStepDelay(GLOBAL_STEP_DELAY);

  // Menerapkan rasio gigi yang baru: 10:1 untuk sendi rotasi
  stepperBase.setReductionRatio(-10.0, 200 * 16); // Base: 10:1, negatif jika arah fisik motor terbalik
  stepperShoulder.setReductionRatio(10.0, 200 * 16); // Shoulder: 10:1
  stepperElbow.setReductionRatio(10.0, 200 * 16); // Elbow: 10:1
  stepperSlider.setReductionRatio(1.0, microstep_per_rev); // Slider: 1:1
  
  radPerMmSlider = (2.0 * M_PI) / pitch_mm_per_rev; // Konversi mm ke radian (untuk konsistensi internal RampsStepper)

  geom.setUseElbowDownSolution(true); // Default IK ke solusi siku ke bawah

  // Offset Kartesian global akan diatur ke 0 di sini, karena FK/IK akan menghitung relatif terhadap origin internalnya.
  // Posisi ROBOT_HOME_X/Y/Z akan menjadi target yang diinginkan dalam sistem koordinat global.
  geom.setCartesianOffset(0.0, 0.0, 0.0); 

  Serial.println("==== Memulai homing semua sumbu ====");
  homingAll(); // Melakukan homing untuk setiap sumbu
  Serial.println("==== Homing selesai. Robot siap menerima perintah ====");

  // --- Mundur dari limit switch setelah homing untuk menghindari kontak terus-menerus ---
  Serial.println("→ Melakukan back-off dari limit switch...");
  int backOffMaxSteps = 5000; // Batas langkah pengaman untuk back-off
  int debounceDelayMs = 50;  // Debounce delay untuk memastikan limit switch benar-benar terlepas

  backOffUntilLimitReleased(stepperBase, backOffMaxSteps, debounceDelayMs);
  backOffUntilLimitReleased(stepperShoulder, backOffMaxSteps, debounceDelayMs);
  backOffUntilLimitReleased(stepperElbow, backOffMaxSteps, debounceDelayMs);
  backOffUntilLimitReleased(stepperSlider, backOffMaxSteps, debounceDelayMs);
  
  Serial.println("→ Back-off selesai.");

  // --- LANGKAH KALIBRASI HOME BARU ---
  // 1. Gerakkan J0 ke -165 derajat dari limit switch home (posisi 0 langkah stepper)
  Serial.println("→ Melakukan gerakan kalibrasi J0 -165 derajat...");
  stepperBase.stepToPositionRad(radians(-165.0)); // Gerakkan ke -165 deg dari 0 langkah stepper
  waitForMovement(); // Tunggu hingga gerakan J0 selesai
  Serial.println("→ Gerakan kalibrasi J0 selesai.");

  // 2. Reset posisi internal stepper ke 0 pada posisi fisik saat ini.
  // Ini mendefinisikan posisi fisik saat ini sebagai '0' langkah untuk setiap stepper.
  Serial.println("→ Mengatur ulang posisi internal stepper ke 0 langkah (posisi fisik saat ini).");
  stepperBase.setPosition(0); 
  stepperShoulder.setPosition(0);
  stepperElbow.setPosition(0);
  stepperSlider.setPosition(0);

  // 3. Set offset nol kinematik.
  // Offset ini akan memastikan bahwa ketika stepper berada di posisi 0 langkah (yaitu, setelah gerakan J0 -165),
  // model kinematika akan menginterpretasikan sudut-sudutnya sebagai 90, -14.00, dan -91.77 derajat.
  // kinematic_angle = physical_angle_from_stepper_zero + kinematic_zero_offset
  // Karena physical_angle_from_stepper_zero sekarang 0, maka kinematic_zero_offset = desired_kinematic_angle.
  geom.setKinematicZeroOffsets(
      radians(90.0),   // Base offset: Ketika stepper Base di 0 langkah, sudut kinematik adalah 90 deg.
      radians(-14.00), // Shoulder offset: Ketika stepper Shoulder di 0 langkah, sudut kinematik adalah -14.00 deg.
      radians(-91.77)  // Elbow offset: Ketika stepper Elbow di 0 langkah, sudut kinematik adalah -91.77 deg.
  );

  // 4. Inisialisasi interpolator ke posisi Kartesian yang diinginkan (ROBOT_HOME_X/Y/Z/E)
  interpolator.setCurrentPos(ROBOT_HOME_X, ROBOT_HOME_Y, ROBOT_HOME_Z, ROBOT_HOME_E);
  Serial.print("Initial interpolator position set to desired ROBOT_HOME: [");
  Serial.print(interpolator.getX()); Serial.print(", ");
  Serial.print(interpolator.getY()); Serial.print(", ");
  Serial.print(interpolator.getZ()); Serial.print(", ");
  Serial.print(interpolator.getE()); Serial.println("]");

  // Debugging: Hitung FK langsung dari sudut kinematik yang diinginkan untuk posisi home target
  // Ini seharusnya mencetak [ROBOT_HOME_X, ROBOT_HOME_Y, ROBOT_HOME_Z] jika IK dan FK bekerja dengan benar.
  float debugFKBaseRad = radians(90.0);
  float debugFKShoulderRad = radians(-14.00);
  float debugFKElbowRad = radians(-91.77);
  geom.calculateFK(debugFKBaseRad, debugFKShoulderRad, debugFKElbowRad); // Panggil FK dengan sudut kinematik yang diinginkan
  float fkX_debug = geom.getFKX() + ROBOT_HOME_E; // Tambahkan kontribusi slider ke X
  float fkY_debug = geom.getFKY();
  float fkZ_debug = geom.getFKZ();

  Serial.print("FK from Desired Kinematic Home Angles (");
  Serial.print(degrees(debugFKBaseRad), 2); Serial.print(", ");
  Serial.print(degrees(debugFKShoulderRad), 2); Serial.print(", ");
  Serial.print(degrees(debugFKElbowRad), 2); Serial.print("): X="); Serial.print(fkX_debug, 2);
  Serial.print(", Y="); Serial.print(fkY_debug, 2);
  Serial.print(", Z="); Serial.print(fkZ_debug, 2);
  Serial.print(" (Expected: ["); Serial.print(ROBOT_HOME_X, 2);
  Serial.print(", "); Serial.print(ROBOT_HOME_Y, 2);
  Serial.print(", "); Serial.print(ROBOT_HOME_Z, 2);
  Serial.println("])"); 

  Serial.println("Robot siap menerima perintah G-code atau J-code.");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (handleDebugCommands(line)) { // Menangani perintah debug (POS, J0, J1, J2, J3)
        return; 
      }
      // Jika bukan perintah debug, coba parsing sebagai G-code (G0, G1, G4, G28, M-code)
      if (command.handleGcodeLine(line)) {
          if (!queue.isFull()) {
              queue.push(command.getCmd());
              Serial.println("OK");
          } else {
              Serial.println("Error: Command queue is full. Please wait.");
          }
      } else {
          Serial.println("Error: Unknown command or G-code format.");
      }
    }
  }

  // Proses perintah dari antrian
  if (!queue.isEmpty() && interpolator.isFinished()) { // Hanya proses jika interpolator selesai
    Cmd cmd = queue.pop();
    executeCommand(cmd); 
  }

  // Jika interpolator sedang berjalan, perbarui posisi dan gerakkan motor
  if (!interpolator.isFinished()) {
    interpolator.updateActualPosition();
    float x_interp = interpolator.getX();
    float y_interp = interpolator.getY();
    float z_interp = interpolator.getZ();
    float e_interp = interpolator.getE(); // Posisi slider

    // Sesuaikan target X untuk IK dengan mengurangi posisi slider
    float x_ik_target = x_interp - e_interp;

    // Gunakan Inverse Kinematics untuk mendapatkan sudut sendi dari posisi Kartesian yang diinginkan
    geom.setPositionCartesianOffset(x_ik_target, y_interp, z_interp); // Hanya X,Y,Z
    
    // Periksa apakah solusi IK valid
    // Di robotGeometry.cpp, jika solusi tidak ditemukan, nilai NaN akan dihasilkan.
    // Kita perlu memeriksa ini.
    if (!isnan(geom.getBaseRad()) && !isnan(geom.getShoulderRad()) && !isnan(geom.getElbowRad())) { 
      float theta1 = geom.getBaseRad();
      float theta2 = geom.getShoulderRad();
      float theta3 = geom.getElbowRad();

      stepperBase.enable(true);
      stepperShoulder.enable(true);
      stepperElbow.enable(true);
      stepperSlider.enable(true);

      float slider_rad = e_interp * radPerMmSlider; // Konversi mm slider ke radian untuk stepper

      stepperBase.stepToPositionRad(theta1);
      stepperShoulder.stepToPositionRad(theta2);
      stepperElbow.stepToPositionRad(theta3);
      stepperSlider.stepToPositionRad(slider_rad);
    } else {
      // Jika IK gagal, hentikan interpolasi dan laporkan error
      Serial.println("Error: Target Kartesian tidak dapat dijangkau. Menghentikan gerakan.");
      interpolator.setCurrentPos(x_interp, y_interp, z_interp, e_interp); // Hentikan interpolasi di posisi saat ini
    }
  }
  
  // Update motor terus-menerus di loop untuk pergerakan halus
  stepperBase.update(); 
  stepperShoulder.update(); 
  stepperElbow.update(); 
  stepperSlider.update(); 

  digitalWrite(LED_PIN, (millis() % 500 < 250) ? HIGH : LOW);
}

void homingAll() {
  // Urutan homing untuk setiap sumbu
  homeAxis(stepperBase);
  Serial.println("→ Homing Base (RAMPS Z-Axis) selesai.");

  homeAxis(stepperShoulder); 
  Serial.println("→ Homing Shoulder (RAMPS Y-Axis) selesai."); 

  homeAxis(stepperElbow); 
  Serial.println("→ Homing Elbow (RAMPS X-Axis) selesai."); 

  homeAxis(stepperSlider);
  Serial.println("→ Homing Slider selesai.");
}

void homeAxis(RampsStepper& stepper) {
  // Aktifkan motor sebelum pergerakan homing
  digitalWrite(stepper.getEnablePin(), LOW); 
  delay(10); // Memberi waktu driver untuk aktif
  
  Serial.print("    Homing stepPin=");
  Serial.print(stepper.getStepPin()); 
  Serial.print(" → menunggu limit switch (Pin ");
  Serial.print(stepper.getLimitPin()); 
  Serial.print(")... Limit state: ");
  Serial.println(digitalRead(stepper.getLimitPin()));

  // Jika sudah pada limit switch, gerakkan menjauh dulu untuk memastikan homing dari luar limit
  if (digitalRead(stepper.getLimitPin()) == LOW) {
    Serial.println("    [Limit switch sudah aktif. Bergerak menjauh dari limit...]");
    // Tentukan arah berlawanan untuk menjauh dari limit
    // Gunakan getReverseDirection() untuk membalik arah pin jika diperlukan
    digitalWrite(stepper.getDirPin(), stepper.getReverseDirection() ? (stepper.getDirHighToHome() ? HIGH : LOW) : (stepper.getDirHighToHome() ? LOW : HIGH)); 
    // Gerakkan beberapa langkah kecil atau sampai limit dilepas
    int backOffStepsInitial = 2000; 
    for (int i = 0; i < backOffStepsInitial && digitalRead(stepper.getLimitPin()) == LOW; i++) {
        stepMotor(stepper.getStepPin());
    }
    delay(100); // Penundaan untuk memastikan pelepasan mekanis limit
    if (digitalRead(stepper.getLimitPin()) == LOW) {
        Serial.println("    WARNING: Tidak dapat bergerak menjauh dari limit switch. Periksa koneksi atau jika robot macet.");
        // Anda mungkin ingin menambahkan logika error handling di sini
    } else {
        Serial.println("    Berhasil bergerak menjauh dari limit switch.");
    }
  }

  // Sekarang, bergerak ke arah limit switch sampai aktif
  // Gunakan getReverseDirection() untuk membalik arah pin jika diperlukan
  digitalWrite(stepper.getDirPin(), stepper.getReverseDirection() ? (stepper.getDirHighToHome() ? LOW : HIGH) : (stepper.getDirHighToHome() ? HIGH : LOW)); 
  
  while (digitalRead(stepper.getLimitPin()) != LOW) { // Loop selama limit BELUM aktif (HIGH)
    stepMotor(stepper.getStepPin()); 
  }
  Serial.println("     [Limit switch aktif]");
  delay(50); // Debounce delay tambahan setelah limit switch terpicu

  // --- PENTING: Atur ulang posisi internal stepper ke 0 ---
  // Ini mendefinisikan posisi fisik motor di limit switch sebagai '0' langkah.
  // Ini adalah titik referensi fisik untuk semua perhitungan langkah berikutnya.
  stepper.setPosition(0); 
  Serial.println("    >> Posisi internal stepper diatur ke 0 (Titik Referensi Home Fisik).");

  // Motor tetap diaktifkan sementara selama urutan homing untuk menjaga posisi.
}

// Fungsi pembantu untuk back-off dari limit switch
void backOffUntilLimitReleased(RampsStepper& stepper, int maxSteps, int debounceDelayMs) {
  stepper.enable(true); // Aktifkan motor untuk pergerakan back-off
  Serial.print("    → Mundur ");
  // Identifikasi nama sumbu untuk pesan log yang lebih jelas
  if (stepper.getStepPin() == ROTATE_STEP_PIN) Serial.print("Base");
  else if (stepper.getStepPin() == SHOULDER_STEP_PIN) Serial.print("Shoulder"); 
  else if (stepper.getStepPin() == ELBOW_STEP_PIN) Serial.print("Elbow");
  else if (stepper.getStepPin() == SLIDER_STEP_PIN) Serial.print("Slider");
  Serial.println(" dari limit switch...");

  // Tentukan arah berlawanan untuk menjauh dari limit
  // Gunakan getReverseDirection() untuk membalik arah pin jika diperlukan
  digitalWrite(stepper.getDirPin(), stepper.getReverseDirection() ? (stepper.getDirHighToHome() ? HIGH : LOW) : (stepper.getDirHighToHome() ? LOW : HIGH)); 

  int stepsMoved = 0;
  // Bergerak menjauh selama limit masih aktif DAN belum mencapai batas langkah maksimum
  while (stepper.isLimitActive() && stepsMoved < maxSteps) {
    stepMotor(stepper.getStepPin());
    stepsMoved++;
  }
  delay(debounceDelayMs); // Debounce delay setelah limit dilepas

  if (stepper.isLimitActive()) {
      Serial.print("    PERINGATAN: ");
      if (stepper.getStepPin() == ROTATE_STEP_PIN) Serial.print("Base");
      else if (stepper.getStepPin() == SHOULDER_STEP_PIN) Serial.print("Shoulder"); 
      else if (stepper.getStepPin() == ELBOW_STEP_PIN) Serial.print("Elbow");
      else if (stepper.getStepPin() == SLIDER_STEP_PIN) Serial.print("Slider");
      Serial.println(" tidak dapat bergerak menjauh dari limit switch. Periksa koneksi atau jika robot macet.");
  } else {
      Serial.print("    ");
      if (stepper.getStepPin() == ROTATE_STEP_PIN) Serial.print("Base");
      else if (stepper.getStepPin() == SHOULDER_STEP_PIN) Serial.print("Shoulder"); 
      else if (stepper.getStepPin() == ELBOW_STEP_PIN) Serial.print("Elbow");
      else if (stepper.getStepPin() == SLIDER_STEP_PIN) Serial.print("Slider");
      Serial.print(" mundur "); Serial.print(stepsMoved); Serial.println(" langkah.");
  }
}

void stepMotor(int pin) { 
  digitalWrite(pin, HIGH);
  delayMicroseconds(GLOBAL_STEP_DELAY); 
  digitalWrite(pin, LOW);
  delayMicroseconds(GLOBAL_STEP_DELAY); 
}

// executeCommand sekarang menangani G0, G1, G4, G28, dan M-code
void executeCommand(const Cmd &cmd) { 
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: // G0: Rapid move (sama seperti G1 tanpa interpolasi halus)
      case 1: { // G1: Linear move
        float targetX = isnan(cmd.valueX) ? interpolator.getX() : cmd.valueX;
        float targetY = isnan(cmd.valueY) ? interpolator.getY() : cmd.valueY;
        float targetZ = isnan(cmd.valueZ) ? interpolator.getZ() : cmd.valueZ;
        float targetE = isnan(cmd.valueE) ? interpolator.getE() : cmd.valueE;
        float feedF  = cmd.valueF; // Kecepatan dalam mm/min

        // Jika feedRate tidak diberikan, gunakan default atau rapid
        if (feedF == 0.0) feedF = 1000.0; // Default rapid feedrate

        interpolator.setInterpolation(targetX, targetY, targetZ, targetE, feedF);
        Serial.print("G"); Serial.print(cmd.num); Serial.print(": Interpolating to X"); Serial.print(targetX);
        Serial.print(" Y"); Serial.print(targetY); Serial.print(" Z"); Serial.print(targetZ);
        Serial.print(" E"); Serial.print(targetE); Serial.print(" F"); Serial.println(feedF);
        break;
      }
      case 28: {
        stepperBase.enable(true);
        stepperShoulder.enable(true);
        stepperElbow.enable(true);
        stepperSlider.enable(true);
        Serial.println("<<< G28 → Homing all axes >>>");
        homingAll();
        // Setelah homing, lakukan back-off seperti di setup
        Serial.println("→ Melakukan back-off dari limit switch setelah G28...");
        int backOffMaxSteps = 5000; 
        int debounceDelayMs = 50;  
        backOffUntilLimitReleased(stepperBase, backOffMaxSteps, debounceDelayMs);
        backOffUntilLimitReleased(stepperShoulder, backOffMaxSteps, debounceDelayMs);
        backOffUntilLimitReleased(stepperElbow, backOffMaxSteps, debounceDelayMs);
        backOffUntilLimitReleased(stepperSlider, backOffMaxSteps, debounceDelayMs);
        Serial.println("→ Back-off setelah G28 selesai.");

        // --- LANGKAH KALIBRASI HOME BARU (Sama seperti di setup()) ---
        // 1. Gerakkan J0 ke -165 derajat dari limit switch home (posisi 0 langkah stepper)
        Serial.println("→ Melakukan gerakan kalibrasi J0 -165 derajat...");
        stepperBase.stepToPositionRad(radians(-165.0)); // Gerakkan ke -165 deg dari 0 langkah stepper
        waitForMovement(); // Tunggu hingga gerakan J0 selesai
        Serial.println("→ Gerakan kalibrasi J0 selesai.");

        // 2. Reset posisi internal stepper ke 0 pada posisi fisik saat ini.
        Serial.println("→ Mengatur ulang posisi internal stepper ke 0 langkah (posisi fisik saat ini).");
        stepperBase.setPosition(0); 
        stepperShoulder.setPosition(0);
        stepperElbow.setPosition(0);
        stepperSlider.setPosition(0);

        // 3. Set offset nol kinematik.
        geom.setKinematicZeroOffsets(
            radians(90.0),   // Base offset: Ketika stepper Base di 0 langkah, sudut kinematik adalah 90 deg.
            radians(-14.00), // Shoulder offset: Ketika stepper Shoulder di 0 langkah, sudut kinematik adalah -14.00 deg.
            radians(-91.77)  // Elbow offset: Ketika stepper Elbow di 0 langkah, sudut kinematik adalah -91.77 deg.
        );

        // 4. Inisialisasi interpolator ke posisi Kartesian yang diinginkan (ROBOT_HOME_X/Y/Z/E)
        interpolator.setCurrentPos(ROBOT_HOME_X, ROBOT_HOME_Y, ROBOT_HOME_Z, ROBOT_HOME_E);
        Serial.println("G28: Homing dan kalibrasi posisi selesai.");
        break;
      }
      case 4: {
        int t_ms = (int)(cmd.valueT * 1000.0);
        Serial.print("G4: Dwell ");
        Serial.print(t_ms);
        Serial.println(" ms");
        delay(t_ms); // Menambahkan delay aktual
        break;
      }
      default:
        Serial.print("Unknown G-code: G");
        Serial.println(cmd.num);
        break;
    }
  }
  else if (cmd.id == 'M') {
    switch (cmd.num) {
      case 3: {
        Serial.println("M3: Gripper ON");
        gripperStepper.setSpeed(200); 
        gripperStepper.step((int)cmd.valueT);
        break;
      }
      case 5: {
        Serial.println("M5: Gripper OFF");
        gripperStepper.setSpeed(200); 
        gripperStepper.step(-((int)cmd.valueT));
        break;
      }
      case 8:
        digitalWrite(SUCTION_PIN, HIGH);
        Serial.println("M8: Suction ACTIVE");
        break;
      case 9:
        digitalWrite(SUCTION_PIN, LOW);
        Serial.println("M9: Suction INACTIVE");
        break;
      case 17:
        Serial.println("M17: Enable all drivers");
        stepperBase.enable(true);
        stepperShoulder.enable(true);
        stepperElbow.enable(true);
        stepperSlider.enable(true);
        break;
      case 18:
        Serial.println("M18: Disable all drivers");
        stepperBase.disable();
        stepperShoulder.disable();
        stepperElbow.disable();
        stepperSlider.disable();
        break;
      case 106:
        Serial.println("M106: Fan ON");
        fan.enable(true);
        break;
      case 107:
        Serial.println("M107: Fan OFF");
        fan.enable(false);
        break;
      default:
        Serial.print("Unknown M-code: M");
        Serial.println(cmd.num);
        break;
    }
  }
}

void waitForMovement(long timeout_ms) {
    Serial.print("    Waiting for movement to finish (timeout: ");
    Serial.print(timeout_ms);
    Serial.println(" ms)...");
    unsigned long start_time = millis();
    while ((stepperBase.isMoving() || stepperShoulder.isMoving() ||
            stepperElbow.isMoving() || stepperSlider.isMoving()) &&
           (millis() - start_time < timeout_ms))
    {
        stepperBase.update();
        stepperShoulder.update();
        stepperElbow.update();
        stepperSlider.update();
        delayMicroseconds(50); 
    }
    if (millis() - start_time >= timeout_ms) {
        Serial.println("    Movement timed out!");
    } else {
        Serial.println("    Movement finished.");
    }
}

// Fungsi untuk Joint Space Control (gerakan langsung per sendi)
void parseAndMoveJoint(const String &line) {
    char jointChar = line.charAt(1); // J0, J1, J2, J3
    float targetValue = line.substring(2).toFloat(); // Ini adalah sudut kinematik yang diinginkan (dalam derajat)

    Serial.print("JOINT_DEBUG >> Moving J");
    Serial.print(jointChar);
    Serial.print(" to ");
    Serial.print(targetValue, 2); Serial.println(" deg (Kinematic Target)"); // Menambahkan presisi untuk debug

    float desiredKinematicRad = radians(targetValue);
    float physicalTargetRad;
    RampsStepper* currentStepper = nullptr; // Pointer ke stepper yang akan digerakkan

    switch (jointChar) {
        case '0': // Base (J0)
            currentStepper = &stepperBase;
            // physical_angle_from_stepper_zero = kinematic_angle - kinematic_zero_offset
            physicalTargetRad = desiredKinematicRad - geom.getKinematicBaseZeroOffsetRad();
            Serial.print("JOINT_DEBUG >> Base Physical Target Rad: "); Serial.println(physicalTargetRad, 4);
            Serial.print("JOINT_DEBUG >> Base Physical Target Deg: "); Serial.println(degrees(physicalTargetRad), 2);
            currentStepper->stepToPositionRad(physicalTargetRad);
            break;
        case '1': // Shoulder (J1)
            currentStepper = &stepperShoulder;
            physicalTargetRad = desiredKinematicRad - geom.getKinematicShoulderZeroOffsetRad();
            Serial.print("JOINT_DEBUG >> Shoulder Physical Target Rad: "); Serial.println(physicalTargetRad, 4);
            Serial.print("JOINT_DEBUG >> Shoulder Physical Target Deg: "); Serial.println(degrees(physicalTargetRad), 2);
            currentStepper->stepToPositionRad(physicalTargetRad);
            break;
        case '2': // Elbow (J2)
            currentStepper = &stepperElbow;
            physicalTargetRad = desiredKinematicRad - geom.getKinematicElbowZeroOffsetRad();
            Serial.print("JOINT_DEBUG >> Elbow Physical Target Rad: "); Serial.println(physicalTargetRad, 4);
            Serial.print("JOINT_DEBUG >> Elbow Physical Target Deg: "); Serial.println(degrees(physicalTargetRad), 2);
            currentStepper->stepToPositionRad(physicalTargetRad);
            break;
        case '3': // Slider (J3 - dalam mm, bukan sudut)
            currentStepper = &stepperSlider;
            // Untuk slider, `targetValue` sudah dalam mm. Konversi ke radian internal
            // (radToStepFactor untuk slider akan mengonversi ini ke langkah).
            // Slider tidak memiliki offset kinematik dalam model ini, jadi langsung gunakan targetValue
            physicalTargetRad = targetValue * radPerMmSlider; 
            Serial.print("JOINT_DEBUG >> Target Slider (mm): "); Serial.println(targetValue, 2);
            Serial.print("JOINT_DEBUG >> Slider Physical Target Rad: "); Serial.println(physicalTargetRad, 4);
            Serial.print("JOINT_DEBUG >> Slider Target Steps: "); Serial.println( (long)(physicalTargetRad * currentStepper->getRadToStepFactor()) );
            currentStepper->stepToPositionRad(physicalTargetRad);
            break;
        default:
            Serial.println("JOINT_DEBUG >> Error: Unknown joint command.");
            return;
    }

    if (currentStepper) {
        currentStepper->enable(true); // Pastikan motor aktif untuk pergerakan
        long single_axis_timeout_ms = 120000; // Timeout default yang cukup besar
        unsigned long start_time_joint = millis();
        while (currentStepper->isMoving() && (millis() - start_time_joint < single_axis_timeout_ms)) {
            currentStepper->update();
            delayMicroseconds(50); 
        }
        if (millis() - start_time_joint >= single_axis_timeout_ms) {
            Serial.println("    Joint Movement timed out!");
        } else {
            Serial.println("    Joint Movement finished.");
        }
    }
}

// handleDebugCommands sekarang menangani perintah POS dan J-code
bool handleDebugCommands(const String &line) {
    String cmd = line;
    cmd.toUpperCase();

    if (cmd.equalsIgnoreCase("POS")) {
        // Dapatkan posisi langkah motor saat ini
        long base_steps = stepperBase.getPosition();
        long shoulder_steps = stepperShoulder.getPosition();
        long elbow_steps = stepperElbow.getPosition();
        long slider_steps = stepperSlider.getPosition();

        // Konversi langkah ke radian fisik untuk sendi rotasi
        float base_rad_physical = base_steps / stepperBase.getRadToStepFactor();
        float shoulder_rad_physical = shoulder_steps / stepperShoulder.getRadToStepFactor();
        float elbow_rad_physical = elbow_steps / stepperElbow.getRadToStepFactor();
        float slider_mm = slider_steps * (pitch_mm_per_rev / microstep_per_rev);

        // Hitung posisi Kartesian menggunakan Forward Kinematics
        // calculateFK sudah memperhitungkan kinematicZeroOffsets secara internal
        geom.calculateFK(base_rad_physical, shoulder_rad_physical, elbow_rad_physical);
        float fkX = geom.getFKX() + slider_mm; // Tambahkan kontribusi slider ke X secara manual
        float fkY = geom.getFKY();
        float fkZ = geom.getFKZ();

        Serial.println("--- Current Robot State ---");
        Serial.print("POS>> Steps [Base, Shoulder, Elbow, Slider]= [");
        Serial.print(base_steps);
        Serial.print(", ");
        Serial.print(shoulder_steps);
        Serial.print(", ");
        Serial.print(elbow_steps);
        Serial.print(", ");
        Serial.print(slider_steps);
        Serial.println("]");

        // Tampilkan sudut kinematik (sudah memperhitungkan offset)
        Serial.print("      Angles (Radians) [Base, Shoulder, Elbow]= [");
        Serial.print(base_rad_physical + geom.getKinematicBaseZeroOffsetRad(), 4); 
        Serial.print(", ");
        Serial.print(shoulder_rad_physical + geom.getKinematicShoulderZeroOffsetRad(), 4); 
        Serial.print(", ");
        Serial.print(elbow_rad_physical + geom.getKinematicElbowZeroOffsetRad(), 4); 
        Serial.println("]");

        Serial.print("      Angles (Degrees) [Base, Shoulder, Elbow]= [");
        Serial.print(degrees(base_rad_physical + geom.getKinematicBaseZeroOffsetRad()), 2); 
        Serial.print(", ");
        Serial.print(degrees(shoulder_rad_physical + geom.getKinematicShoulderZeroOffsetRad()), 2); 
        Serial.print(", ");
        Serial.print(degrees(elbow_rad_physical + geom.getKinematicElbowZeroOffsetRad()), 2); 
        Serial.println("]");
        
        Serial.print("      End-Effector (Cartesian mm) [X,Y,Z]= [");
        Serial.print(fkX, 2);
        Serial.print(", ");
        Serial.print(fkY, 2);
        Serial.print(", ");
        Serial.print(fkZ, 2);
        Serial.println("]");

        Serial.print("   Slider (mm)= ");
        Serial.println(slider_mm, 3);
        Serial.println("---------------------------");
        return true;
    }

    // DEBUG FUNCTION: Move physical motors directly (Joint Space Control)
    if (cmd.startsWith("J0") || cmd.startsWith("J1") || cmd.startsWith("J2") || cmd.startsWith("J3")) {
        parseAndMoveJoint(line);
        return true;
    }

    // Untuk GOTO (IK)
    if (cmd.startsWith("GOTO")) {
        // Ekstrak X, Y, Z, E dari perintah GOTO
        float targetX = NAN, targetY = NAN, targetZ = NAN, targetE = NAN;
        String temp = line;
        temp.toUpperCase();

        int idxX = temp.indexOf('X');
        int idxY = temp.indexOf('Y');
        int idxZ = temp.indexOf('Z');
        int idxE = temp.indexOf('E');

        if (idxX >= 0) targetX = temp.substring(idxX + 1).toFloat();
        if (idxY >= 0) targetY = temp.substring(idxY + 1).toFloat();
        if (idxZ >= 0) targetZ = temp.substring(idxZ + 1).toFloat();
        if (idxE >= 0) targetE = temp.substring(idxE + 1).toFloat();

        // Jika ada nilai yang tidak diberikan, gunakan posisi interpolator saat ini
        if (isnan(targetX)) targetX = interpolator.getX();
        if (isnan(targetY)) targetY = interpolator.getY();
        if (isnan(targetZ)) targetZ = interpolator.getZ();
        if (isnan(targetE)) targetE = interpolator.getE();

        Serial.print("IK_DEBUG >> Moving to [X,Y,Z,E] = [");
        Serial.print(targetX);
        Serial.print(", ");
        Serial.print(targetY);
        Serial.print(", ");
        Serial.print(targetZ);
        Serial.print(", ");
        Serial.print(targetE);
        Serial.println("]");

        // Panggil IK untuk mendapatkan sudut sendi yang diperlukan
        // Sesuaikan target X untuk IK dengan mengurangi posisi slider
        float x_ik_target = targetX - targetE;
        geom.setPositionCartesianOffset(x_ik_target, targetY, targetZ); 
        
        // Periksa apakah solusi IK valid
        if (!isnan(geom.getBaseRad()) && !isnan(geom.getShoulderRad()) && !isnan(geom.getElbowRad())) { 
            float theta1 = geom.getBaseRad();
            float theta2 = geom.getShoulderRad();
            float theta3 = geom.getElbowRad();

            stepperBase.enable(true);
            stepperShoulder.enable(true);
            stepperElbow.enable(true);
            stepperSlider.enable(true);

            float slider_rad = targetE * radPerMmSlider;

            stepperBase.stepToPositionRad(theta1);
            stepperShoulder.stepToPositionRad(theta2);
            stepperElbow.stepToPositionRad(theta3);
            stepperSlider.stepToPositionRad(slider_rad);

            waitForMovement();
            interpolator.setCurrentPos(targetX, targetY, targetZ, targetE); // Perbarui posisi interpolator
        } else {
            Serial.println("    IK_DEBUG >> ERROR: Target position out of reach!");
        }
        return true;
    }
    return false;
}
// parseAndMoveFK and parseAndMoveIK functions are now replaced by the logic inside handleDebugCommands
