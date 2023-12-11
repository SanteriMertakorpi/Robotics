from controller import Robot, Camera
import cv2
import numpy as np

# Initialize the Webots robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#Assign the LED-lights on the puck for use later
led1 = robot.getDevice('led0')
led2 = robot.getDevice('led1')

speedometer= robot.getDevice('display')
speedometer.setFont('Arial',10,1)


#Same for the speaker
speaker = robot.getDevice('speaker')

# Get the camera device and enable it
camera = robot.getDevice("camera")
camera.enable(timestep)  # Enable the camera with a sampling period of once every timestep
camera.recognitionEnable(timestep)



# Set motor devices for left and right wheels
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set motor speeds
max_speed = 6.28  # Maximum motor speed
base_speed = 2  # Base speed for straight-line motion

# Set proportional, integral, and derivative gains
Kp = 0.01  #Lower if robot overshoots on corrective turns
Ki = 0.0001 #eliminating long-term drift and bias, lower if unstable
Kd = 0.001 #reducing oscillations

# Define integral saturation limits
integral_max = 0.5
integral_min = -0.5

# Define variables for tracking
last_error = 0
integral = 0

def detect_line_position(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to create a binary image where the line is white
    _, thresholded = cv2.threshold(gray, 185, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours,_ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
    
    
    #Updating the speedometer
    #There's no way to clear the display so we redraw a rectangle that fills the display,
    #then draw text
    speedometer.setColor(000000)
    speedometer.fillRectangle(0,0,speedometer.getWidth(),speedometer.getHeight())
    
    speedometer.setColor(111111)
    speed=str(base_speed)
    speedometer.drawText(('Current speed: '+speed),40,5)
    
    if(camera.getRecognitionNumberOfObjects() != 0): #Check if camera sees an object
    
    #Assign the first object from the recognized objects array to a value
        firstObject = camera.getRecognitionObjects()[0] 
        if(firstObject is not None):
            
            purpose=firstObject.getModel() #finding out which block is seen
            
        
            if(purpose=='slowdown'):      #If seen block is red, slow down and play metallica          
                base_speed=1.5
                led2.set(1)
                speaker.playSound(speaker,speaker,'goslow.wav',0.5,1,0,0)
            elif(purpose=='acceleration'):
                base_speed=3                
                led1.set(1)
                speaker.playSound(speaker,speaker,'gofast.wav',0.5,1,0,0)
                #If seen block is green, speed up and play eurobeat
                
    else:        
        speaker.stop()   #Base condition, do this if nothing except the line is visible
        base_speed=2
        led1.set(0)
        led2.set(0)  
            
    
    # Convert the camera image to a NumPy array for line detection
    image_data = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

    # Call the line position detection algorithm
    line_x = detect_line_position(image_data)

    if line_x is not None:
        # Calculate the error based on the line's position
        error = line_x - (camera.getWidth() / 2)

        # Calculate the integral of the error (use it to reduce build-up of incorrect movements)
        integral += error

        # Calculate the derivative of the error (use it to reduce overshooting)
        derivative = error - last_error

        # Calculate motor speeds based on PID control
        left_speed = base_speed + (Kp * error + Ki * integral + Kd * derivative)
        right_speed = base_speed - (Kp * error + Ki * integral + Kd * derivative)

        # Limit the integral term
        integral = max(integral_min, min(integral_max, integral))
        
        #The above snippet is important if the track has many sections which cause errors

        # Update the last error
        last_error = error

    else:
        # No line detected, do a little spin to find one
        left_speed = 2
        right_speed = 0

    # Ensure the speeds are within bounds
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    # Set motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    
    
    #Difficulties:
    #Constructing a world which is usable by the camera
    #-(obviously) using just the camera for input data
    #-Movement combined with directional steering
    #-getting even somewhat accurate turns in corners
    #-Configuring the camera node so it can actually support object recognition
    #-Adding the display to read current speed in real-time
    #
