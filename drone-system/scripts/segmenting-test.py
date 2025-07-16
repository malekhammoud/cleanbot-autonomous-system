import cv2
import numpy as np

# Read the image
image = cv2.imread('images/image1.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
blur = cv2.medianBlur(hsv, 11)

# Define the color range for masking
lower = np.array([27, 3, 107])
upper = np.array([98, 102, 255])

# Create a mask and apply it to the image
mask = cv2.inRange(blur, lower, upper)
res = cv2.bitwise_and(image, image, mask=mask)

# Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw bounding boxes around each detected contour
for contour in contours:
    if cv2.contourArea(contour) > 10:  # Adjust area threshold as needed
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw rectangle

# Show the original image with bounding boxes
cv2.imshow("Detected Litter", image)
cv2.imshow("Mask", mask)
cv2.imshow('Stack', np.hstack([image, res]))
cv2.waitKey(0)
cv2.destroyAllWindows()

