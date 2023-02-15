######################################################################################
# Program Name: wall_bouncer.py
# Written by: Will Ward
#
# Control program for Raspberry Pi powered roomba-like robot.
#
# Program Flow:
#   1. Enable the motors. Confirm the motors are enabled (check GPIO pins voltage level). 
#   2. Robot enters **Pausing** mode. Set the GREEN LED to be dimmer using PWM. Make sure
#      robot's motion is stopped in this mode.
#   3. Press the button to enable the **Playing** mode, and the robot moves forward.
#      Green LED stays lit to indicate **Playing** mode. Use two distance sensors to 
#      check for obstacles. Turn the robot away from obstacles if it gets too close.
#   4. Press the button again to switch the back to **Pausing** mode. 
#   5. Pressing the button switches the mode back and forth. 
#   6. Record time consumption in **Playing** mode. After 60 seconds of play, yellow LED 
#      turns on. After 90 seconds, red blinks 10 times, then the robot shuts down.
# 
########################################################################################


# Import python packages
import time
from gpiozero import LED, PWMLED, Button, DistanceSensor, PhaseEnableRobot


##################
# Preperations   
##################

# Initialize GPIO pins, LEDs, button, distance sensor and the robot 

motor_pin_1 = LED(22)  # motor 1 enable pin set to GPIO 22
motor_pin_2 = LED(23)  # motor 2 enable pin set to GPIO 23

red = PWMLED(10, frequency = 2)  # red LED set to GPIO 10
green = PWMLED(19, frequency = 1000)  # green LED set to GPIO 19
yellow = LED(8) # yellow LED set to GPIO 8 (no PWM)

button = Button(27, hold_time=3)  # Button 1 set to GPIO 27
sensor1 = DistanceSensor(echo=16, trigger=17, max_distance = 4)  # initialize ultrasonic sensor 1
sensor2 = DistanceSensor(echo=18, trigger=14, max_distance = 4)  # max distance in meters

robot = PhaseEnableRobot(left=(24, 12), right=(25,13))  # set GPIO pins for (direction, PWM) on each motor


# Initiate variables such as mode, duty cycles, run time

state = False  # play mode = True, pause mode = False
run_time = 0.0  # keeps track of total time in play mode
red_time = 0.0  # keeps track of total time red is blinking
duty_cycle = list(range(0, 101)) + list(range(100, -1, -1))  # duty cycle for green goes from 0 -> 101 -> -1
i = 0  # used to increment duty_cycle


# Enable the motors and confirm they are on

motor_pin_1.on()  # enable motor1
motor_pin_2.on()  # enable motor2

if (motor_pin_1.is_active):  # check motor1
    print("Motor 1 is on")
else:
    print("Error - Motor 1 is off")

if (motor_pin_2.is_active):  # check motor2
    print("Motor 2 is on")
else:
    print("Error - Motor 2 is off")

time.sleep(0.1) # pause for 0.1 seconds





###########
# Main
###########

try: # loop continuously until ctrl-c is pressed
    while True:
          if button.is_pressed: # if button pressed, switch mode
                print("Play/Pause pressed")
                button.wait_for_release()  # waits for you to release button
                print("Play/Pause released")
                state = not state   # change mode

          
          if state: # if in Playing mode
                green.value = 1   # light up GREEN (full brightness)
                robot.forward(0.7)  # move robot forward (fraction of full power)
                
                run_time += 0.02  # update run_time
                print("ON ")
                print(run_time)
                
                if run_time > 60:
                    yellow.on()  # light up YELLOW if play mode for 1 minute
                if run_time > 90:
                    red.value = 0.5  # light up RED if play mode for more than 90 seconds
                    red_time += 0.02  # increment red on time
                    if red_time > 10:  # if RED on mroe than 10 seconds, break
                        break
                
                if sensor1.distance < 0.6 or sensor2.distance < 0.6:  # if distance between robot and obstacle < 0.6
                    robot.left(0.7) # turn robot to the left
                    
                time.sleep(.02) # pause for 0.02 seconds


          else:  # otherwise, robot is in pause mode               
                green.value = duty_cycle[i] / 100   # change GREEN's duty cycle
                i += 1
                if i >= len(duty_cycle):
                    i = 0

                robot.stop() # turn robot off
                print("OFF ")
                #print(duty_cycle[i])
                time.sleep(.02)
    
except KeyboardInterrupt:
    # Turn off all LEDs, the robot, and the motors
    red.off()
    yellow.off()
    green.off()
    robot.stop()
    motor_pin_1.off()
    motor_pin_2.off()
    print("\nLEDs and Motors are turned off.")

finally:
    # Ensure that all LEDS, the robot, and the motors are turned off
    red.off()
    yellow.off()
    green.off()
    robot.stop()
    motor_pin_1.off()
    motor_pin_2.off()
    print("\nLEDs and Motors are turned off.")
