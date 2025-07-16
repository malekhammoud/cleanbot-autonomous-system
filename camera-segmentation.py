from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np
import warnings

# Suppress the libpng warning
warnings.filterwarnings("ignore", category=UserWarning, module='libpng')

# Initialize the camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)

# Start the camera
picam2.start()
time.sleep(5)  # Allow the camera to adjust

# Capture the image
picam2.capture_file("test.jpg")
time.sleep(1)

# Load the captured image
image = cv2.imread("test.jpg")

# Check if the image was loaded successfully
if image is None:
    print("Error: Could not load image.")
    exit()

# Example: Detect color (for litter detection)
# Define color range for detection (e.g., red color)
lower_color = np.array([0, 0, 100])  # Lower bound for red
upper_color = np.array([100, 100, 255])  # Upper bound for red

# Convert the image to HSV color space (if needed)
# image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a mask for the specific color range
mask = cv2.inRange(image, lower_color, upper_color)

# Find contours of detected regions
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw bounding boxes around detected litter
for contour in contours:
    if cv2.contourArea(contour) > 100:  # Adjust area threshold as needed
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)

# Save the processed image with detected litter
cv2.imwrite("detected_litter.jpg", image)

# Optionally, start recording video
picam2.start_and_record_video("test.mp4", duration=5)

