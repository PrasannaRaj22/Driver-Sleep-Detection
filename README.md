# Driver-sleep-Detection
his project implements a Driver Sleep and Drowsiness Detection System using an Arduino, a camera (webcam), and Python. The system is designed to monitor a driver’s face and detect signs of drowsiness or sleep. If the driver is detected to be drowsy or asleep, an alert is triggered via a buzzer and LED connected to an Arduino. This is a basic system to help improve road safety by preventing accidents caused by driver fatigue.
Components Used:

    Arduino (Uno or any compatible board)
        Controls the buzzer and LED for sleep/drowsiness alerts.
        Communicates with the Python program via serial.

    Webcam (for face and eye detection)

    3-pin Buzzer
        Connected to the Arduino to trigger sound alerts.

    LED
        Connected to the Arduino for visual alerts.

    Python Libraries:
        OpenCV: Used for facial detection and eye blink recognition.
        dlib: For facial landmark detection (used to track the eyes for blinking).
        imutils: For face detection and manipulation.
        numpy: Used for calculating distances between facial landmarks.

    Arduino Libraries:
        LiquidCrystal: Used for controlling an LCD screen (optional) connected to Arduino for displaying system messages.

How It Works:
1. Arduino Code:

The Arduino code is responsible for receiving commands from the Python program through serial communication. It controls an LED and a buzzer to provide visual and auditory alerts when the driver is detected to be either asleep or drowsy.

    Buzzer and LED Control: When the system detects the driver is asleep ('a'), the buzzer and LED are activated. If the driver is awake or the system is reset ('b'), the buzzer and LED are turned off.
    The LCD screen provides real-time feedback about the system’s state (e.g., "Please wake up Sir.." when asleep, "All Ok" when active).

2. Python Code:

The Python code uses OpenCV and dlib to detect the driver’s face and track the eyes to detect blinks. If the driver’s eyes are detected to be closed for a prolonged period (indicative of sleep), the system sends a command to the Arduino to activate the buzzer and LED.

    Drowsiness Detection: Based on eye blink frequency, the system determines whether the driver is active, drowsy, or asleep.
    Blinked Eyes Detection: Eye blink ratio is calculated by measuring the distance between specific facial landmarks (eyes). A lower blink rate or prolonged closed eyes indicate sleep or drowsiness.

Key States:

    Active: The driver is awake and alert, and no alerts are triggered.
    Drowsy: The driver is showing signs of drowsiness, triggering a visual and auditory alert.
    Sleeping: The driver’s eyes are closed for an extended period, triggering an urgent alert.

Setup and Installation:
1. Arduino Setup:

    Connect the LED and Buzzer to the Arduino according to the pin numbers specified in the code:
        Buzzer to pin 11
        LED to pin 8
    Connect an LCD screen (optional) for feedback display. The LCD uses pins 2, 3, 4, 5, 6, 7.
    Upload the Arduino code to your Arduino board using the Arduino IDE.

2. Python Setup:

    Ensure you have Python 3.x installed on your computer.
    Install the required Python libraries using pip:

    pip install opencv-python dlib imutils numpy

    Download the file shape_predictor_68_face_landmarks.dat from dlib's facial landmarks model and place it in the same directory as your Python script.
    Modify the serial communication port (COM8 in the code) to match the port where your Arduino is connected.

3. Running the System:

    Once both the Arduino and Python code are set up, run the Python script to start the face and eye tracking:

    python driver_sleep_detection.py

    The system will start detecting faces and eyes via the webcam. It will monitor the driver's eye blinks and send alerts to the Arduino when necessary.

Files:

    Arduino Code (arduino_driver_sleep_detection.ino):
        The code that runs on the Arduino to control the buzzer, LED, and LCD.

    Python Code (driver_sleep_detection.py):
        The Python code for detecting drowsiness and sleep based on eye blink rate.

    Model File (shape_predictor_68_face_landmarks.dat):
        The pre-trained model used by dlib for detecting facial landmarks.

How to Use:

    Connect the Arduino to your computer and upload the Arduino code.
    Install the required Python dependencies and ensure the shape_predictor_68_face_landmarks.dat file is available.
    Run the Python script to begin the driver sleep/drowsiness detection.
    The system will provide feedback (on both the LCD and through the buzzer/LED) if the driver is detected to be drowsy or sleeping.
