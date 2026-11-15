
import cv2
import time

# Try camera index 0 first
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows

if not cap.isOpened():
    print("Camera could not be opened.")
    exit()

print("Camera opened successfully.")

# Allow camera initialization
time.sleep(2)

while True:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("No frame received from camera.")
        break

    print("Frame:", frame.shape, end="\r")

    cv2.imshow("Webcam Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
