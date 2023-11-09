"""This code captures video from a webcam, resizes it to 300x300, converts 
   it to grayscale, and displays the video stream with the acquisition 
   frames per second (fps) shown on the display window."""

import cv2

# Initialize the video capture from the default camera (usually the integrated webcam)
#cv2.OPENCV_LOG_LEVEL=debug
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open the camera.")
    exit()

# Define the window name and font settings for displaying fps
window_name = 'Webcam Video Capture'
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
font_color = (255, 255, 255)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read a frame.")
        break
    # Resize the frame to 300x300
    frame = cv2.resize(frame, (300, 300))
    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get the acquisition fps
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Display the fps on the frame
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), font, font_scale, font_color, 1)

    # Display the original and grayscale frames
    cv2.imshow(window_name, frame)

    # Break the loop when the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
