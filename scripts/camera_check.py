import cv2

def check_connected_cameras():
    num_cameras = 5  # Anda dapat menyesuaikan jumlah kamera yang akan diperiksa sesuai dengan kebutuhan
    for i in range(num_cameras):
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            print(f"Tidak dapat membuka kamera dengan ID {i}")
        else:
            print(f"Kamera dengan ID {i} berhasil terbuka")
            cap.release()

check_connected_cameras()