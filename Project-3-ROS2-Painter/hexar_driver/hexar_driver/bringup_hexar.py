######################################################################################
# Program Name: bringup_hexar.py
# Written by: Will Ward
#
# Drive the motors of a physical hardware robot using Twist messages published over
# a ROS 2 topic. The  robot waits to move until it recieves communication from a ROS 2
# node that controls the motion of a simulated turtle robot. The real robot and turtle
# robot are designed to mimic each other's movements. However, since the robots drive
# at different speeds, their motion is not synchronized. Instead, the hardware robot
# has preprogrammed driving commands. The robot waits until it recieves a message from 
# the turtle robot before it starts driving. When placed on the ground, the robot 
# traces out the letter "U". (Note that the turtle continues to draw the letters "CA")
#
########################################################################################



# Import python packages
from gpiozero import LED, PhaseEnableRobot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep


class HexarDriver(Node, PhaseEnableRobot):

    def __init__(self):
        super().__init__('hexar_driver')
        self.subscription = self.create_subscription( # ROS 2 subscriber node
            Twist, # msg type: Twist
            'turtle1/cmd_vel', # topic name
            self.listener_callback, # function to execute when spun
            1)
        self.subscription # prevent unused variable warning
        self.robot = PhaseEnableRobot(left=(24,12), right=(25,13)) # set GPIO pins for (direction, PWM) on each motor
        self.linear_x = 0.0  # variable for storing linear x displacement
        self.angular_z = 0.0  # variable for storing angular z rotation

    def listener_callback(self, msg): # executes when a 'turtle1/cmd_vel' message is recieved
        print('msg.linear.x: ', msg.linear.x)
        print('msg.angular.z: ', msg.angular.z)
        print('--------------')
        self.linear_x = msg.linear.x  # store the message in a variable
        self.angular_z = msg.angular.z  # store the message in a variable

    def waiting_linear(self):
        rclpy.spin_once(self)  # check for a 'turtle1/cmd_vel' message
        if (self.linear_x == 0.0):  # if no message was recieved
            return False 
        else:  # if message was recieved
            return True

    def waiting_angular(self):
        rclpy.spin_once(self) # check for a 'turtle1/cmd_vel' message
        if (self.angular_z == 0.0):  # if no message was recieved
            return False
        else:  # if message was recieved
            return True

    def get_to_distance(self, time):
        # Check if a linear displacement measurement was recieved
        check = False
        while (check == False): # loop until linear displacement message is recieved
            check = self.waiting_linear()

        # Once linear message is recieved, move forward a set amount of time
        self.robot.forward(0.3)  # drive forward
        print('moving forward time: ', time)
        sleep(time)  # pause while the robot drives
        self.robot.stop()  # robot stops driving
    
    def get_to_angle(self, time):
        # Check if an angular rotation measurement was recieved
        check = False
        while (check == False):  # loop until angular rotation message is recieved
            check = self.waiting_angular()
       
        # once angular command is recieved, rotate robot for a set amount of time
        self.robot.left(0.3)  # turn robot to the left
        print('turning left time: ', time)
        sleep(time)  # pause while the robot rotates
        self.robot.stop()  # robot stops rotating




def main(args=None):
    rclpy.init(args=args)

    hexar_driver = HexarDriver()  # initialze the HexarDriver class
    motor_pin_1 = LED(22)  # motor 1 enable pin set to GPIO 22                           
    motor_pin_2 = LED(23)  # motor 2 enable pin set to GPIO 23
    motor_pin_1.on()  # enable motor1
    motor_pin_2.on()  # enable motor2

    
    #####################
    # Draw the Letter U #
    #####################

    # Stage 1: Get to the starting point of the U
    hexar_driver.get_to_angle(0.9)  # rotate for 0.9 seconds
    hexar_driver.get_to_distance(2.0)  # drive forward for 2 seconds
    
    # Stage 2: Draw the left side of the U
    hexar_driver.get_to_angle(0.65)  # rotate for 0.65 seconds
    hexar_driver.get_to_distance(1.5)  # drive forward for 1.5 seconds

    # Stage 3: Draw the bottom of the U
    hexar_driver.get_to_angle(0.5)  # rotate for 0.5 seconds
    hexar_driver.get_to_distance(0.85)  # drive forward for 0.85 seconds

    # Stage 4: Draw the right of the U
    hexar_driver.get_to_angle(0.5)  # rotate for 0.5 seconds
    hexar_driver.get_to_distance(1.6)  # drive forward for 1.6 seconds

    # Stage 5: Get to the C
    hexar_driver.get_to_angle(1.6)  # rotate for 1.6 seconds
    hexar_driver.get_to_distance(2.5)  # drive forward for 2.5 seconds


    hexar_driver.robot.stop()  # stop the robot
    hexar_driver.destroy_node()  # destroy the ROS 2 node
    rclpy.shutdown()



if __name__ == '__main__':
    main()
