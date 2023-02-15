#############################################################
# Uses RPi.GPIO library to slowly blink an LED connected to
# to pin 18 of a Raspberry Pi.
#
# Lights up the LED gradually in 2 seconds, then turns it off
# gradually in 1 seconds. Loop continuously until stopped.
############################################################


# import python packages
import time
import RPi.GPIO as GPIO

# Configure GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup GPIO Pin for PWM
GPIO.setup(18, GPIO.OUT) # set GPIO pin 18 to be an output pin
pwm = GPIO.PWM(18, 100) # set the PWM of pin 18 to 100 Hz freq

# Initialize the duty cycle (dc) of pin 18
dc = 0
pwm.start(dc)

# Loop until you press ctl C
try:  
    while True:
        for dc in range(0, 100, 3): # increase dc from 0 to 100 in steps of 3
            pwm.ChangeDutyCycle(dc) # update the duty cycle
            time.sleep(0.05)  # pause for 0.05 seconds
            print(dc)
        for dc in range(100, 0, -5):  # decrease dc from 100 to 5 in steps of 5
            pwm.ChangeDutyCycle(dc) # update the duty cycle
            time.sleep(0.05)  # pause for 0.05 seconds
            print(dc)
except KeyboardInterrupt:
    print("Ctl C pressed - ending program")

# Before program ends,         
pwm.stop()  # stop PWM
GPIO.cleanup() # reset GPIO ports
    