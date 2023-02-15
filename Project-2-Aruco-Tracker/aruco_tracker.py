######################################################################################
# Program Name: aruco_tracker.py
# Written by: Will Ward
#
# Programs a RPi powered robot to follow ArUco markers to navigate through an area.
#
# Program Flow:
#   1. Define a robot control class to include two new functions: 
#      foward_left(), and foward_right().
#   2. Initialize OpenCV and ArUco detection
#   3. Read an image
#   4. If a marker is detected in the image, drive towards the center of the marker
#   5. If a marker is not detected for a set amount of time, the robot is lost and
#      will slowly turn to around to locate an ArUco marker
# 
########################################################################################


# Import python packages
from gpiozero import LED, PhaseEnableRobot
import cv2
from time import sleep


# Robot control class that improves PhaseEnableRobot
class BetterRobot(PhaseEnableRobot):

    def left_forward(self, max_speed = 1, speed=1):
        # Make the robot turn left while still moving forward by running
        # the right motor faster than the left motor.
        self.right_motor.forward(max_speed) 
        self.left_motor.forward(speed)
 
    def right_forward(self, max_speed = 1, speed=1):
        # Make the robot turn right while still moving forward by running
        # the right motor faster than the left motor.
        self.right_motor.forward(speed)
        self.left_motor.forward(max_speed)
    
    def compute_center(self, corners=[]):
        # Computes the center of the image.
        center_x = (1/2)*(corners[0][0][0][0] + corners[0][0][2][0])
        #print('top left corner = ', corners[0][0][0][0])
        #print('bottom right corner = ', corners[0][0][2][0])
        return center_x
    
    

# Initialize the robot and enable the motors
robot = BetterRobot(left=(24, 12), right=(25,13)) # set GPIO pins for (direction, PWM) on each motor
motor_pin_1 = LED(22)  # motor 1 enable pin set to GPIO 22                           
motor_pin_2 = LED(23)  # motor 2 enable pin set to GPIO 23
motor_pin_1.on()  # enable motor1
motor_pin_2.on()  # enable motor2


# Setup video capture and set video properties
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()
cap = cv2.VideoCapture(0)


#Initialize other variables
l = 635  # pixel length of a captured image
h = 317  # approximately half the pixel length of a captured image
s = 0.6  # max speed of the wheels (60% of full throttle)
center_x = 0  # location of the center of the aruco marker
wait = 25  # if the robot doesn't see an aruco marker for 25 seconds, it's lost and needs to turn
lost = False  # false means the robot isn't lost yet
iteration = 0  # counts to 1000 before turning
location = 0  # keeps track of the last place the marker was seen (center_x)





###########
# Main
###########

while True:
    ret, img = cap.read() # capture an image from the camera
    
    # Aruco Detection
    (corners, ids, rejects) = cv2.aruco.detectMarkers(  # look for aruco markers in the image
        img,
        arucoDict,
        parameters=arucoParams
    )
    cv2.aruco.drawDetectedMarkers(  # draw the aruco marker on the captured image
        image=img,
        corners=corners,
        ids=ids
        
    )
    cv2.imshow("detection", img)  # display the image

    # Determine if the robot is lost
    if iteration == wait:  # if the robot hasn't seen a marker for "wait" time
        lost = True  # change state to lost

    if corners == []:  # If no aruco marker was detected
        #print('NO IMAGE!')
        if lost == True:  # if the robot is lost (has not seen a marker lately)
            if iteration == wait:
                #print(location)
                if location < (l/2): # if the marker was last seen on the left half of the screen
                    robot.left(0.3)  # robot turns left
                    sleep(0.3)  # pause while the robot turns
                    iteration = 0
                else:  # if the marker was last seen on the right half of the screen
                    robot.right(0.3)  # robot turns right
                    sleep(0.3)  # pause while the robot turns
                    iteration = 0
                    
            else:
                robot.stop()
        else:  # if the robot does not see an image but is not lost yet
            robot.forward(0.3)  # robot moves forward
            #print(iteration)
        
        iteration = iteration + 1
        
    else:  # if a marker was detected
        lost = False  # the robot is no longer lost
        iteration = 0   # reset the lost time counter (because the robot is not lost anymore)
        center_x = robot.compute_center(corners)  # calculate the position of the aruco's center
        location = center_x  # store the most recent location of center_x
        #print(location)
        #print(center_x)
        
        # Make the robot drive towards the center of the marker
        # The speed of each wheel depends on the position from the center
        if center_x < h:  # if the marker is on the left half of the screen
            robot.left_forward(s, (s/l)*(center_x) + (s/2))  # drive forward and to the left
            print(center_x, '          left')
            print(s, '   ', (s/l)*(center_x) + (s/2))
        
        elif center_x >= h: # if the marker is on the right half of the screen
            robot.right_forward(s, (-s/(2*h))*(center_x - h) + s) # drive forward and to the right
            print(center_x, '           right')
            print(s, '   ', (-s/(2*h))*(center_x - h) + s) 
    
    
    # Break out of the loop if "q" is pressed
    if cv2.waitKey(1) == ord("q"):
        break


# Stop the robot
robot.stop() 

# Stop openCV
cap.release()
cv2.destroyAllWindows()
