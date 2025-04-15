import cv2
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('COM4', 115200, timeout=1)  # Adjust 'COM4' to the correct port
time.sleep(2)  # Wait for Arduino to initialize

# Function to send commands to Arduino
def send_command(command):
    arduino.write(f"{command}\n".encode())
    arduino.flush()  # Ensure the command is sent immediately
    print(f"Sent to Arduino: {command}")

# Function to log serial data received from Arduino
def log_serial_data():
    while arduino.in_waiting > 0:  # Check if there's data in the serial buffer
        try:
            response = arduino.readline().decode('utf-8', errors='ignore').strip()  # Ignore invalid characters
            if response:  # If there is any data, print and log it
                print(f"Received from Arduino: {response}")
        except UnicodeDecodeError:
            print("UnicodeDecodeError: Could not decode response from Arduino.")

# Function to simulate sending a stop signal to the Arduino
def send_stop_signal():
    send_command("SHUTDOWN")
    time.sleep(1)  # Give time for the Arduino to process
    log_serial_data()  # Log the response after sending the stop command

# Function to simulate sending a move forward signal to the Arduino
def send_move_forward_signal():
    send_command("FORWARD")
    time.sleep(1)  # Move forward for 1 second
    log_serial_data()  # Log the response after sending the forward command
    send_command("STOP")
    log_serial_data()  # Log the response after stopping

# Function to capture video from the webcam and detect stop and forward signs
def capture_video():
    cap = cv2.VideoCapture(0)  # Capture video from the default webcam
    
    # Load the Haar Cascade classifiers for stop sign and forward sign detection
    stop_sign_cascade = cv2.CascadeClassifier('stop_sign.xml')
    forward_sign_cascade = cv2.CascadeClassifier('left-sign.xml')  # Load forward sign Haar Cascade
    
    stop_sign_detected = False  # Flag to track if stop sign was detected
    forward_sign_detected = False  # Flag to track if forward sign was detected
    
    while True:
        ret, frame = cap.read()  # Read a frame from the video stream
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the frame to grayscale
        
        # Detect stop signs in the frame
        stop_signs = stop_sign_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        
        # Detect forward signs in the frame
        forward_signs = forward_sign_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        
        # Process detected stop signs
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw rectangle
            send_stop_signal()  # Send stop signal to Arduino
            print("Stop sign detected! System shutting down.")
            stop_sign_detected = True  # Set flag to True to prevent further actions
            break
        
        # Process detected forward signs
        if not forward_sign_detected:
            for (x, y, w, h) in forward_signs:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw rectangle
                send_move_forward_signal()  # Send move forward signal to Arduino
                forward_sign_detected = True  # Set flag to prevent further detections
                break
        
        # Display the processed frame
        cv2.imshow('Video Feed', frame)

        # Continuously log any serial data from Arduino (non-blocking)
        log_serial_data()
        
        # Exit the loop if 'q' is pressed or stop sign is detected
        if cv2.waitKey(1) & 0xFF == ord('q') or stop_sign_detected:
            break
    
    cap.release()  # Release the video capture object
    cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == "__main__":
    capture_video()
