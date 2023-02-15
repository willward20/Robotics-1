##################################################################################
# Turn on LEDs using GPIO pins and a push button. Use PWM to make LEDs appear
# to blink and slowly turn on and off.
# 
# Workflow:
#    1. Light up LEDs one by one in order: RED for 1 second >> YELLOW for 2 seconds 
#       >> BLUE for 3 seconds >> GREEN for 4 seconds >> blink ALL in 2Hz for 2 seconds.
#    2. Slowly blink GREEN using PWM (2 seconds gradually on and 2 seconds gradually off).
#       Press B1, GREEN stays on. Press B1 again, GREEN gets back to PWM blinking mode.
#    3. Count time consumption of GREEN stays on (GO). DO NOT count GREEN blink (GB) time.
#    4. If GO takes more than 10 seconds, light up YELLOW (other LED will remain their status). 
#    5. If GO takes more than 20 seconds, blink RED in 2 Hz for 10 seconds. Then, shutdown the
#       system (pull down all involved GPIOs).
####################################################################################


# Import python packages
import time
import RPi.GPIO as GPIO

# Configure GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup GPIO pins for output and input
chan_list = [18, 23, 12, 16]               
GPIO.setup(chan_list, GPIO.OUT)  # set up four LED output channels
GPIO.setup(25, GPIO.IN)  # set up the push button input channel


# Light up the LEDS in order
GPIO.output(18, True)  #turn on red
time.sleep(1)  #wait 1 second
GPIO.output(23, True)  #turn on yellow
time.sleep(2)  #wait 2 seconds
GPIO.output(12, True)  #turn on blue
time.sleep(3)  #wait 3 seconds
GPIO.output(16, True)  #turn on green
time.sleep(4)  #wait 4 seconds

# turn off all LEDS
GPIO.output(18, False)  
GPIO.output(23, False)
GPIO.output(12, False)
GPIO.output(16, False)


# Setup output GPIO pins for PWM (frequency = 2 Hz)
red = GPIO.PWM(18, 2)  # GPIO 18 (red)
yellow = GPIO.PWM(23, 2)  # GPIO 23 (yellow)
blue = GPIO.PWM(12, 2)  #  GPIO 12 (blue) 
green = GPIO.PWM(16, 2)  #setup GPIO 16 (green)


# Initialize each LED with a duty cycle of 20%
red.start(20)     
yellow.start(20)    
blue.start(20)      
green.start(20)     

time.sleep(2)  # sleep for 2 seconds

# Stop all LEDs
red.stop() 
yellow.stop()
blue.stop()
green.stop()

time.sleep(2) # sleep for 2 seconds



#2. Slowly blink GREEN use PWM (2 seconds gradually on and 2 seconds gradually off).
#   Press B1, GREEN stays on. Press B1 again, GREEN gets back to PWM blinking mode.

# initialize variables
state = False  # if state is True, light on mode. If state is False, blink mode
counter = 0.0  # increments each time the loop repeats on light on mode
seconds = 0  # number of seconds passed since light mode turned on
dc = 0  # duty cycle of green LED
up = True  # up is True if green needs to slowly blink on. up is false if green needs to slowly blink off

red.ChangeFrequency(2)  # change red PWM freq to 2 Hz
red.ChangeDutyCycle(1)  # change duty cyle of red to 1
red_counter = 0.0  # number of iterations of red blinking since it started
red_on = False  # red_on is true if red is blinking AND you're in green on mode

yellow.start(0)  # yellow starts with dc of 0
green.start(0)  # green starts with dc of 0


while True:   # infinite loop unless broken from the inside
    if seconds == 10:  # if green on mode is on for 10 seconds, turn on yellow
        yellow.ChangeDutyCycle(100)
    if seconds == 20:  # if green on mode is on for 20 seconds, start blinking red
        red.start(1)
        red_on = True
    
    if not GPIO.input(25):  # detect if button is pushed
        state = not state  # change modes (light on or blink)
        print(state)
        time.sleep(0.02)
    else:
        if state == True:  # if you're in light on mode
            green.ChangeDutyCycle(100)  # dc is 100 --- green stays on constantly
            print(state)
            print(seconds)
            counter = counter + 1  # increment for each iteration
            if counter == 50:   # for every 50 iterations, 1 second passes
                seconds = seconds + 1
                counter = 0
        
            if red_on == True: # if red is on, keep track
                red_counter = red_counter + 1
            if red_counter == 500:  # if 500 iterations pass, red has been on for 10 seconds
                break  # break out of the while loop
        
        else:  # slow blinking mode 
            if up == True:  # green is slowly turning on
                dc = dc + 0.5  # update duty cycle
                green.ChangeDutyCycle(dc)
            else:  # green is slowly turning off
                dc = dc - 0.5  # update duty cycle
                green.ChangeDutyCycle(dc)
            print(state)
            print(dc)
            if dc == 0: 
                up = True  # green needs to start turning on
            if dc == 100:
                up = False  # green needs to start turning off
            
        time.sleep(0.02)
    
    
# Turn off all LEDs
red.stop()  
yellow.stop()
blue.stop()
green.stop()

# Turn off all GPIO pins
GPIO.output(18, False) 
GPIO.output(23, False)
GPIO.output(12, False)
GPIO.output(16, False)

    
