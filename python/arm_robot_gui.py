import sys
import cv2
import serial
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QPushButton, QComboBox, QMessageBox, QFrame, QTextEdit, QSizePolicy, QLineEdit
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPainterPath
from PyQt5.QtCore import Qt, QTimer

# Pilihan algoritma: False = HSV/HoughCircles, True = YOLO
USE_YOLO = True

if USE_YOLO:
    from ultralytics import YOLO

class SimpleRobotCamApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arm-Robot Control")
        # Set ukuran window 1920x1080
        self.setGeometry(100, 100, 1920, 1080)

        self.serial_port = None
        # Ambil kamera langsung tanpa tombol connect/disconnect
        self.capture = cv2.VideoCapture(0)
        if not self.capture.isOpened():
            QMessageBox.warning(self, "Error", "Tidak dapat membuka kamera")
            self.capture = None

        # Inisialisasi YOLO jika diperlukan
        if USE_YOLO:
            try:
                self.yolo_model = YOLO("best.pt")  # Ganti dengan path weight custom jika perlu
                # Definisikan kelas yang akan digunakan
                self.class_names = {0: 'Buah_Hijau', 1: 'Buah_Jingga', 2: 'Buah_Kuning', 3: 'Buah_Merah'}
                # Warna untuk setiap kelas (BGR)
                self.class_colors = {
                    0: (0, 255, 0),      # Hijau
                    1: (0, 165, 255),    # Jingga
                    2: (0, 255, 255),    # Kuning
                    3: (0, 0, 255)       # Merah
                }
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Gagal memuat YOLO model: {e}")
                self.yolo_model = None
        else:
            self.yolo_model = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.process_frame)
        self.latest_detection = "Menunggu deteksi..."
        self.last_detected_class = None  # Menyimpan kelas terakhir yang terdeteksi
        self.detection_results = []  # Menyimpan semua hasil deteksi
        
        # Robot state variables
        self.robot_ready = False  # Siap menerima perintah
        self.robot_status = "Idle"  # Kondisi kerja

        # UI Setup
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        central.setStyleSheet("background-color: #1e1e1e;")  # Dark background

        # ---------- Bagian kiri: Kamera (3/4) ----------
        camera_widget = QWidget()
        camera_layout = QVBoxLayout(camera_widget)
        camera_layout.setContentsMargins(10, 10, 10, 10)
        camera_layout.setSpacing(5)

        # Label judul kamera di atas panel kamera
        camera_title = QLabel("Gambar Kamera")
        camera_title.setStyleSheet("color: #ffffff; font-size: 24px; font-weight: bold;")
        camera_title.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(camera_title)

        # Panel kamera dengan bezel rounded, mengisi sisa ruang
        self.camera_label = QLabel()
        self.camera_label.setStyleSheet(
            "background-color: #000000; border-radius: 20px;"
        )
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        camera_layout.addWidget(self.camera_label)

        main_layout.addWidget(camera_widget, 3)

        # ---------- Bagian kanan: Kontrol ----------
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(20, 10, 20, 10)
        right_layout.setSpacing(20)

        # Card: Hasil Deteksi
        detect_card = QFrame()
        detect_card.setStyleSheet(
            "background-color: #2a2a2a; border-radius: 15px;"
        )
        detect_layout = QVBoxLayout(detect_card)
        detect_layout.setContentsMargins(15, 15, 15, 15)
        detect_layout.setSpacing(10)
        detect_title = QLabel("Deteksi Gambar")
        detect_title.setStyleSheet("color: #ffffff; font-size: 20px; font-weight: bold;")
        detect_layout.addWidget(detect_title)
        self.detect_text = QTextEdit()
        self.detect_text.setReadOnly(True)
        self.detect_text.setStyleSheet(
            "background-color: rgba(255,255,255,30); color: #ffffff; font-size: 18px; padding: 10px; border-radius: 8px;"
        )
        self.detect_text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.detect_text.setText(self.latest_detection)
        detect_layout.addWidget(self.detect_text)
        # Tombol Start/Stop
        btn_layout = QHBoxLayout()
        self.detect_button = QPushButton("Detect")
        self.detect_button.setFixedHeight(60)
        self.detect_button.setStyleSheet(
            "background-color: #28A745; color: #ffffff; font-size: 16px; font-weight: bold; border-radius: 8px;"
        )
        self.detect_button.clicked.connect(self.start_detection)
        btn_layout.addWidget(self.detect_button)
        self.go_button = QPushButton("Go")
        self.go_button.setFixedHeight(60)
        self.go_button.setStyleSheet(
            "background-color: #FFC107; color: #000000; font-size: 16px; font-weight: bold; border-radius: 8px;"
        )
        self.go_button.clicked.connect(self.send_detection)
        btn_layout.addWidget(self.go_button)
        detect_layout.addLayout(btn_layout)
        right_layout.addWidget(detect_card, 3)

        # Card: Kontrol Komunikasi
        comm_card = QFrame()
        comm_card.setStyleSheet(
            "background-color: #2a2a2a; border-radius: 15px;"
        )
        comm_layout = QVBoxLayout(comm_card)
        comm_layout.setContentsMargins(15, 15, 15, 15)
        comm_layout.setSpacing(10)
        comm_title = QLabel("Kontrol Komunikasi Robot")
        comm_title.setStyleSheet("color: #ffffff; font-size: 20px; font-weight: bold;")
        comm_layout.addWidget(comm_title)
        port_layout = QHBoxLayout()
        port_label = QLabel("Port:")
        port_label.setStyleSheet("color: #ffffff; font-size: 16px;")
        port_layout.addWidget(port_label)
        self.port_combo = QComboBox()
        com_ports = [f"COM{i}" for i in range(1, 11)]
        self.port_combo.addItems(com_ports)
        self.port_combo.setStyleSheet(
            "background-color: rgba(255,255,255,30); color: #ffffff; font-size: 16px; padding: 8px; border-radius: 8px;"
        )
        port_layout.addWidget(self.port_combo)
        comm_layout.addLayout(port_layout)
        baud_layout = QHBoxLayout()
        baud_label = QLabel("Baud:")
        baud_label.setStyleSheet("color: #ffffff; font-size: 16px;")
        baud_layout.addWidget(baud_label)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"] )
        self.baud_combo.setStyleSheet(
            "background-color: rgba(255,255,255,30); color: #ffffff; font-size: 16px; padding: 8px; border-radius: 8px;"
        )
        baud_layout.addWidget(self.baud_combo)
        comm_layout.addLayout(baud_layout)
        self.connect_button = QPushButton("Hubungkan Robot")
        self.connect_button.setFixedHeight(60)
        self.connect_button.setStyleSheet(
            "background-color: #007ACC; color: #ffffff; font-size: 16px; font-weight: bold; border-radius: 8px;"
        )
        self.connect_button.clicked.connect(self.connect_robot)
        comm_layout.addWidget(self.connect_button)
        right_layout.addWidget(comm_card, 1)

        # Card: Status Robot
        status_card = QFrame()
        status_card.setStyleSheet(
            "background-color: #2a2a2a; border-radius: 15px;"
        )
        status_layout = QVBoxLayout(status_card)
        status_layout.setContentsMargins(15, 15, 15, 15)
        status_layout.setSpacing(10)
        status_title = QLabel("Status Robot")
        status_title.setStyleSheet("color: #ffffff; font-size: 20px; font-weight: bold;")
        status_layout.addWidget(status_title)
        self.ready_label = QLabel("Ready: Tidak Siap")
        self.ready_label.setStyleSheet("color: #ffffff; font-size: 16px;")
        status_layout.addWidget(self.ready_label)
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("color: #ffffff; font-size: 16px;")
        status_layout.addWidget(self.status_label)
        right_layout.addWidget(status_card, 1)

        # Card: Perintah Manual
        manual_card = QFrame()
        manual_card.setStyleSheet(
            "background-color: #2a2a2a; border-radius: 15px;"
        )
        manual_layout = QVBoxLayout(manual_card)
        manual_layout.setContentsMargins(15, 15, 15, 15)
        manual_layout.setSpacing(10)
        manual_title = QLabel("Kirim Perintah Manual")
        manual_title.setStyleSheet("color: #ffffff; font-size: 20px; font-weight: bold;")
        manual_layout.addWidget(manual_title)
        input_layout = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Masukkan kode, misal j0")
        self.cmd_input.setStyleSheet(
            "background-color: rgba(255,255,255,30); color: #ffffff; font-size: 16px; padding: 8px; border-radius: 8px;"
        )
        input_layout.addWidget(self.cmd_input)
        self.send_button = QPushButton("Send")
        self.send_button.setFixedHeight(50)
        self.send_button.setStyleSheet(
            "background-color: #17A2B8; color: #ffffff; font-size: 16px; font-weight: bold; border-radius: 8px;"
        )
        self.send_button.clicked.connect(self.send_manual)
        input_layout.addWidget(self.send_button)
        manual_layout.addLayout(input_layout)
        right_layout.addWidget(manual_card, 1)
        right_layout.addStretch(1)

        main_layout.addWidget(right_widget, 1)
        
        # Nonaktifkan tombol Go awalnya
        self.go_button.setEnabled(False)

    def connect_robot(self):
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.robot_ready = True
            self.robot_status = "Idle"
            self.update_status()
            QMessageBox.information(self, "Info", f"Tersambung ke {port} @ {baud}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Gagal koneksi: {e}")

    def update_status(self):
        self.ready_label.setText(f"Ready: {'Siap' if self.robot_ready else 'Tidak Siap'}")
        self.status_label.setText(f"Status: {self.robot_status}")

    def start_detection(self):
        if not self.capture:
            QMessageBox.warning(self, "Error", "Kamera tidak tersedia.")
            return
        
        # Reset deteksi sebelumnya
        self.last_detected_class = None
        self.latest_detection = "Mendeteksi..."
        self.detect_text.setText(self.latest_detection)
        self.go_button.setEnabled(False)
        
        # Jalankan deteksi satu frame
        self.timer.start(30)  # jalankan deteksi satu kali

    def send_detection(self):
        if not self.robot_ready:
            QMessageBox.warning(self, "Error", "Robot belum terhubung.")
            return
            
        if self.last_detected_class is None:
            QMessageBox.warning(self, "Error", "Belum ada hasil deteksi.")
            return
            
        # Tentukan kode perintah berdasarkan kelas
        if self.last_detected_class == 0:  # Hijau
            code = "P1"
        elif self.last_detected_class == 2:  # Kuning
            code = "P2"
        elif self.last_detected_class == 3:  # Merah
            code = "P3"
        else:
            QMessageBox.warning(self, "Error", "Kelas tidak dikenali")
            return
            
        try:
            self.robot_status = "Running"
            self.update_status()
            self.serial_port.write(code.encode() + b"\n")
            QMessageBox.information(self, "Info", f"Perintah {code} dikirim ke robot")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Gagal mengirim perintah: {e}")
        finally:
            # Setelah 2 detik, kembalikan status ke Idle
            QTimer.singleShot(2000, lambda: self.set_robot_idle())
            
    def set_robot_idle(self):
        self.robot_status = "Idle"
        self.update_status()

    def send_manual(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            QMessageBox.warning(self, "Error", "Masukkan kode perintah.")
            return
            
        if self.serial_port and self.serial_port.is_open:
            try:
                self.robot_status = "Running"
                self.update_status()
                self.serial_port.write(cmd.encode() + b"\n")
                QMessageBox.information(self, "Info", f"Perintah {cmd} dikirim ke robot")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Gagal mengirim: {e}")
            finally:
                # Setelah 2 detik, kembalikan status ke Idle
                QTimer.singleShot(2000, lambda: self.set_robot_idle())
        else:
            QMessageBox.warning(self, "Error", "Robot belum terhubung.")

    def process_frame(self):
        if not self.capture:
            return
            
        # Hanya proses satu frame
        self.timer.stop()
        
        ret, frame = self.capture.read()
        if not ret:
            return
            
        # Simpan frame asli untuk deteksi
        orig_frame = frame.copy()
        
        # Dapatkan ukuran untuk tampilan
        lbl_w = self.camera_label.width()
        lbl_h = self.camera_label.height()
        
        # Reset deteksi sebelumnya
        self.last_detected_class = None
        detected_label = "Tidak ada objek terdeteksi"
        detection_found = False

        if USE_YOLO and self.yolo_model:
            # Gunakan YOLO untuk deteksi pada frame asli
            results = self.yolo_model(orig_frame)[0]
            
            # Reset hasil deteksi
            self.detection_results = []
            
            # Gambar semua deteksi dengan warna sesuai kelas
            for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
                cls = int(cls)
                conf = float(conf)
                
                # Hanya proses kelas yang diinginkan (0, 2, 3)
                if cls in [0, 2, 3]:
                    # Simpan hasil deteksi
                    self.detection_results.append((box, cls, conf))
                    
                    # Simpan deteksi pertama untuk perintah
                    if not detection_found:
                        self.last_detected_class = cls
                        detected_label = f"Terdeteksi: {self.class_names[cls]} (conf: {conf:.2f})"
                        detection_found = True
        
        # Resize frame untuk ditampilkan
        disp = cv2.resize(orig_frame, (lbl_w, lbl_h))
        
        # Hitung rasio skala
        scale_x = lbl_w / orig_frame.shape[1]
        scale_y = lbl_h / orig_frame.shape[0]
        
        # Gambar bounding box pada frame yang diresize
        for box, cls, conf in self.detection_results:
            # Koordinat bounding box asli
            x1, y1, x2, y2 = map(int, box)
            
            # Skala koordinat ke ukuran tampilan
            x1_scaled = int(x1 * scale_x)
            y1_scaled = int(y1 * scale_y)
            x2_scaled = int(x2 * scale_x)
            y2_scaled = int(y2 * scale_y)
            
            # Gambar bounding box dengan warna sesuai kelas
            color = self.class_colors.get(cls, (0, 255, 0))
            cv2.rectangle(disp, (x1_scaled, y1_scaled), (x2_scaled, y2_scaled), color, 3)
            
            # Label dengan nama kelas dan confidence
            label = f"{self.class_names[cls]} {conf:.2f}"
            
            # Ukuran teks
            font_scale = max(0.8, min(lbl_w, lbl_h) / 1000)
            thickness = max(1, int(min(lbl_w, lbl_h) / 500))
            
            # Ukuran background label
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
            )
            
            # Pastikan posisi label tidak keluar dari frame
            label_y = max(30, y1_scaled - 10)
            label_y = min(lbl_h - text_height - 10, label_y)
            
            # Gambar background label
            cv2.rectangle(
                disp, 
                (x1_scaled, label_y - text_height - 10), 
                (x1_scaled + text_width + 10, label_y), 
                color, 
                -1
            )
            
            # Gambar teks
            cv2.putText(
                disp, 
                label, 
                (x1_scaled + 5, label_y - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                font_scale, 
                (0, 0, 0),  # Warna teks hitam
                thickness
            )

        # Update tampilan
        self.latest_detection = detected_label
        self.detect_text.setText(self.latest_detection)
        
        # Tampilkan frame di GUI
        rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        orig_pix = QPixmap.fromImage(qimg)

        rounded = QPixmap(w, h)
        rounded.fill(Qt.transparent)
        painter = QPainter(rounded)
        painter.setRenderHint(QPainter.Antialiasing)
        path = QPainterPath()
        path.addRoundedRect(0, 0, w, h, 20, 20)
        painter.setClipPath(path)
        painter.drawPixmap(0, 0, orig_pix)
        painter.end()
        self.camera_label.setPixmap(rounded)
        
        # Jika terdeteksi, aktifkan tombol Go
        self.go_button.setEnabled(detection_found)

    def closeEvent(self, event):
        if self.timer.isActive():
            self.timer.stop()
        if self.capture:
            self.capture.release()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimpleRobotCamApp()
    window.show()
    sys.exit(app.exec_())