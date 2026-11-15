
import cv2

# 0 is usually the default webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not access webcam")
    exit()

print("Webcam started. Press 'q' to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        break

    cv2.imshow("Live Webcam Feed", frame)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
