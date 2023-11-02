from controller import Robot, Camera
import cv2
import numpy as np

# Initialize the Webots robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())
# Get the camera device and enable it
camera = robot.getDevice("camera")
camera.enable(timestep)  # Enable the camera with a sampling period of 32ms


# Set motor devices for left and right wheels
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set motor speeds
max_speed = 6.28  # Maximum motor speed
base_speed = 2  # Base speed for straight-line motion

def detect_line_position(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to create a binary image where the line is white
    _, thresholded = cv2.threshold(gray, 185, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store line position
    line_x = None

    if contours:
        # Find the largest contour (assuming it's the white line)
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the centroid of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            line_x = int(M["m10"] / M["m00"])

    return line_x

# Main control loop
while robot.step(timestep) != -1:
    # Capture an image from the camera
    image = camera.getImage()
    
    # Convert the camera image to a NumPy array for line detection
    image_data = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    
    # Call the line position detection algorithm
    line_x = detect_line_position(image_data)
    
    if line_x is not None:
        if line_x <260 and line_x > 220:
            left_speed = base_speed
            right_speed = base_speed
            
        elif line_x > 260:
            left_speed = base_speed
            right_speed = base_speed*0.5
        elif line_x < 220:
            left_speed = base_speed*0.5
            right_speed = base_speed
            
        # Calculate an error based on the line's position
        error = line_x - (camera.getWidth() / 2)
        
        # Implement a proportional control to adjust the wheel speeds
        #left_speed = base_speed - error / (camera.getWidth() / 2)
        #right_speed = base_speed + error / (camera.getWidth() / 2)
    else:
        # No line detected, move forward
        left_speed = 0
        right_speed = base_speed
    
    # Ensure the speeds are within bounds
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))
    
    # Set motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    
    print("Line position:", line_x)
    print("Left speed:", left_speed)
    print("Right Speed:", right_speed)
    print("Error:", error)
# Cleanup

