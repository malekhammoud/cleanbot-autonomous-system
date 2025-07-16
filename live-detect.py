from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(camera_config)
picam2.start()

# Detect litter in a video frame
def detect_litter(frame, detection_count):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv, 11)

    # Define the color range for masking (adjust for litter detection)
    lower = np.array([27, 3, 107])
    upper = np.array([98, 102, 255])

    # Create a mask and apply it to the image
    mask = cv2.inRange(blur, lower, upper)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contour is detected as litter
    for contour in contours:
        if cv2.contourArea(contour) > 10:  # Adjust area threshold
            detection_count += 1  # Increment the detection count
            print(f"Detected at {detection_count}")  # Print detection message
            return detection_count  # Return updated count

    return detection_count  # No litter detected, return current count

def main():
    detection_count = 0

    try:
        while True:
            # Get camera frame
            frame = picam2.capture_array()

            # Check for litter detection in the current frame
            detection_count = detect_litter(frame, detection_count)

            # Optional: Show the frame with mask (for debugging)
            # cv2.imshow("Frame", frame)
            # cv2.imshow("Mask", mask)
            
            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

