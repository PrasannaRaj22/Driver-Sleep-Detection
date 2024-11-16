# Importing necessary libraries
import cv2
import numpy as np
import dlib
from imutils import face_utils
import serial
import time

# Initialize serial communication with Arduino
s = serial.Serial('COM8', 9600)

# Initializing the camera and taking the instance
cap = cv2.VideoCapture(0)

# Initializing the face detector and landmark detector
hog_face_detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Status markers
sleep = 0
drowsy = 0
active = 0
status = ""
color = (0, 0, 0)

def compute(ptA, ptB):
    dist = np.linalg.norm(ptA - ptB)
    return dist

def blinked(a, b, c, d, e, f):
    up = compute(b, d) + compute(c, e)
    down = compute(a, f)
    ratio = up / (2.0 * down)

    # Check if eyes are blinked
    if ratio > 0.25:
        return 2
    elif ratio > 0.21 and ratio <= 0.25:
        return 1
    else:
        return 0

def sound_alert():
    """Send commands to Arduino to activate/deactivate buzzer and LED"""
    s.write(b'a')  # Turn on buzzer/LED
    time.sleep(2)  # Keep them on for 2 seconds
    s.write(b'b')  # Turn off buzzer/LED

while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = hog_face_detector(gray)
    for face in faces:
        x1 = face.left()
        y1 = face.top()
        x2 = face.right()
        y2 = face.bottom()
        face_frame = frame.copy()
        cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        landmarks = predictor(gray, face)
        landmarks = face_utils.shape_to_np(landmarks)

        # Detect blinks for both eyes
        left_blink = blinked(landmarks[36], landmarks[37], landmarks[38], landmarks[41], landmarks[40], landmarks[39])
        right_blink = blinked(landmarks[42], landmarks[43], landmarks[44], landmarks[47], landmarks[46], landmarks[45])

        # Judge what to do based on eye blinks
        if left_blink == 0 or right_blink == 0:
            sleep += 1
            drowsy = 0
            active = 0
            if sleep > 6:
                sound_alert()  # Activate alert when sleeping
                status = "SLEEPING !!!"
                color = (0, 0, 255)

        elif left_blink == 1 or right_blink == 1:
            sleep = 0
            active = 0
            drowsy += 1
            if drowsy > 6:
                sound_alert()  # Activate alert for drowsiness
                status = "Drowsy !"
                color = (0, 0, 255)

        else:
            drowsy = 0
            sleep = 0
            active += 1
            if active > 6:
                s.write(b'b')  # Send command to turn off the buzzer/LED when active
                status = "Active :)"
                color = (0, 255, 0)

        # Display the status on the screen
        cv2.putText(frame, status, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        # Draw facial landmarks
        for n in range(0, 68):
            (x, y) = landmarks[n]
            cv2.circle(face_frame, (x, y), 1, (255, 255, 255), -1)

    # Show the video feed
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == 27:  # Press ESC to exit
        break

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()
