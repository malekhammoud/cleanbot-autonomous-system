from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
#picam2.start_preview(Preview.DRM)
picam2.start()
time.sleep(5)
print("START")
picam2.capture_file("test.jpg")
time.sleep(1)
picam2.start_and_record_video("test.mp4", duration=5)
