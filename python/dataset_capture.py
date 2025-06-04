import cv2
import os
import time

# Konfigurasi
SAVE_DIR = "dataset"
CAMERA_INDEX = 0        # 0 untuk webcam default
FPS_CAPTURE = 2         # 4 frame per detik
DURATION_SEC = 5       # durasi pengambilan gambar
TOTAL_IMAGES = FPS_CAPTURE * DURATION_SEC

# Membuat folder jika belum ada
os.makedirs(SAVE_DIR, exist_ok=True)

# Inisialisasi kamera
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("❌ Tidak dapat membuka kamera.")
    exit()

print(f"📸 Mulai mengambil {TOTAL_IMAGES} gambar...")
start_time = time.time()

for i in range(TOTAL_IMAGES):
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Gagal menangkap frame.")
        break

    filename = os.path.join(SAVE_DIR, f"img_{i+1:04d}_v3.jpg")
    cv2.imwrite(filename, frame)
    print(f"[{i+1:03d}/{TOTAL_IMAGES}] Tersimpan: {filename}")

    time.sleep(1 / FPS_CAPTURE)

# Selesai
cap.release()
print("✅ Selesai mengambil gambar.")
