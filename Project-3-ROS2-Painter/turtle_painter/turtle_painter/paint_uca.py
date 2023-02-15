######################################################################################
# Program Name: paint_uca.py
# Written by: Will Ward
#
# Use ROS 2 to control the movement of a simulated turtle. Velocity commands are published
# over the 'turtle1/cmd_vel' topic to rotate the turtle and drive it forward. The 
# 'turtle1/set_pen' topic was used to turn the pen on and off (so that turtle draws while
# it drives). The turtle traces out the letters "UCA".
#
########################################################################################


# Import python packages
from turtlesim.srv import SetPen
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan
from time import sleep


class TurtlePainter(Node):
    
    def __init__(self, state):
        super().__init__('turtle_painter')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)  # ROS 2 publisher node (Twist message)
        self.subscription = self.create_subscription(  # ROS 2 subscriber node
            Pose, # msg type: Pose
            'turtle1/pose', # topic name
            self.listener_callback, # function to execute when spun
            1)
        self.subscription  # prevent unused variable warning
        
        # Enable turtle to draw as it moves in the simulation
        self.cli = self.create_client(SetPen, 'turtle1/set_pen')  # ROS 2 client node for SetPen
        while not self.cli.wait_for_service(timeout_sec=1.0):  # wait for the service to be available
                self.get_logger().info('service not available, waiting again...')
        
        # Define object variables
        self.req = SetPen.Request()  # turn on the drawing pen
        self.req.r = 255  # set pen color
        self.req.g = 255  # set pen color
        self.req.b = 255  # set pen color
        self.req.width = 3  # set pen line width
        self.req.off = state  # retermines if pen is turned on or off
        self.cli.call_async(self.req)
        
        self.msg_x = 0.0  # stores x compnent of the Pose message
        self.msg_y = 0.0  # stores y compnent of the Pose message
        self.msg_theta = 0.0  # stores theta compnent of the Pose message
        self.angle_found = False  # True if the turtle reached the desired angle
        
        # Set variables for PID control
        self.current_distance = 0.0  # PID
        self.previous_error = 0.0  # use these for PD control if successful
        self.reference_distance = 0.0  # the distance we want to travel 
        
        
    def listener_callback(self, msg): # executes when a 'turtle1/cmd_vel' message is recieved
        # save the message components in object variables
        self.msg_x = msg.x
        self.msg_y = msg.y
        self.msg_theta = msg.theta
        

    def turtle_publish(self, lin_x, ang_z): # publisher node for turtle1/cmd_vel topic
        msg = Twist()  # set the message type: Twist
        msg.linear.x = lin_x  # set linear x velocity
        msg.linear.y = 0.0  # set linear y velocity
        msg.linear.z = 0.0  # set linear z velocity
        msg.angular.x = 0.0  # set angular x velocity
        msg.angular.y = 0.0  # set angular y velocity
        msg.angular.z = ang_z  # set angular z velocity 	
        self.publisher_.publish(msg) # publish the message to the topic

        
        
    def get_to_point(self, get_to_x, get_to_y, ang_vel): 
        
        # travel to the position (get_to_x, get_to_y)
        
        # Calcualte the distance between current position and destination
        diff_x = get_to_x - self.msg_x # how far in x direction to travel
        diff_y = get_to_y - self.msg_y # how far in y direction to travel
        print('get_to_x: ', get_to_x)
        print('self.msg_x: ', self.msg_x)
        print('diff_x: ', diff_x)
        print('---')
        print('get_to_y: ', get_to_y)
        print('self.msg_y: ', self.msg_y)
        print('diff_y: ', diff_y)
        print('---')

        # Calcualte the angular position needed for the turtle to face the destination point
        if (diff_x >= 0): # if the x destination is to the right of the current x
                theta_desired = atan(diff_y / diff_x) # calculate the angle facing destination
        elif (diff_x < 0 and diff_y > 0): # if the x destination is to the left of current position
                                          # and the y destination is above the current y position
                theta_desired = 3.14159 + atan(diff_y / diff_x) # calculate the angle facing destination
        elif (diff_x < 0 and diff_y < 0): # otherwise
                theta_desired = -3.14159 + atan(diff_y / diff_x) # calculate the angle facing destination
        print('theta_desired: ', theta_desired)
        print('self.msg_theta: ', self.msg_theta)
        print('---')
        

        # Rotate the turtle so it is facing the destination
        if (self.angle_found == False):  # if the turtle is not at the correct angular position
                check =  TurtlePainter.rotation(self, ang_vel, theta_desired)  # rotate the turtle
        else:  # otherwise, the turtle is at the correct angle
                check = True

        # Drive the turtle forward until it reaches the destination
        if (check == True):
                # Calculate the distance turtle needs to travel
                distance_travel = (diff_x**2 + diff_y**2)**0.5 
                print('distance_travel: ', distance_travel)
                
                # Publish a velocity message to drive the turtle forward over 'turtle1/cmd_vel' topic
                # The magnitude of the velocity depends on how close the turtle is to the destination
                if (distance_travel > 0.5): 
                        TurtlePainter.turtle_publish(self, 2.0, 0.0)  
                elif (distance_travel > 0.1):
                        TurtlePainter.turtle_publish(self, 0.2, 0.0)   
                elif (distance_travel > 0.01):
                        TurtlePainter.turtle_publish(self, 0.05, 0.0)      
                else: # if the turtle is close enough to the destination
                        TurtlePainter.turtle_publish(self, 0.0, 0.0)  # turtle stops moving forward
                        self.angle_found = False  # set angle found false for the next stage
                        return True  # indicate that the destination was reached
                        
        return False  # indicate that the destination has not been reached yet




    def pid_get_to_point(self, get_to_x, get_to_y, ang_vel):

        # Use a PID controller to move the turtle to a destination

        check = False
        
        # Rotate the turtle so it faces the destination
        if (self.angle_found == False):  # if the turtle is not facing the destination

                # Calcualte the distance between current position and destination
                diff_x = get_to_x - self.msg_x # how far in x to travel
                diff_y = get_to_y - self.msg_y # how far in y to travel
                 
                # Calcualte the angular position needed for the turtle to face the destination point
                if (diff_x >= 0): # if the desired x is to the right of the current x
                        theta_desired = atan(diff_y / diff_x) # calculate the angle facing destination
                elif (diff_x < 0 and diff_y > 0): # if the desired x is to the left of current
                                                  # and the desired y is above the current y
                        theta_desired = 3.14159 + atan(diff_y / diff_x) # calculate the angle facing destination
                elif (diff_x < 0 and diff_y < 0): # otherwise
                        theta_desired = -3.14159 + atan(diff_y / diff_x) # calculate the angle facing destination
                check =  TurtlePainter.rotation(self, ang_vel, theta_desired) # rotate the turtle

        else:  # if the turtle is facing the destination
                check = True


        # Use PID to drive the turtle to the destination
        if (check == True): # if the turtle is facing the destiantion, drive 
        
                p = -1 #proportionality constant (for P)
                d = 0.1 # proportionality constant (for D)
                
                # Calculate distance between current position and destination
                self.current_distance = ((get_to_x - self.msg_x)**2 + (get_to_y - self.msg_y)**2)**0.5
                print('current_distance = ', self.current_distance)
                
                # Calcualte the error between desired distance from a point and current distance from a point
                distance_error = self.reference_distance - self.current_distance
                change_in_error = distance_error  - self.previous_error # Calcualte the change in error
                print('distance_error: ', distance_error)
                
                # Calcualte how fast the turtle should drive, based on error and change in error
                linear_vel = p*distance_error + d*change_in_error
                
                TurtlePainter.turtle_publish(self, linear_vel, 0.0) # publish the velocity to make the turtle drive
                
                self.previous_error = distance_error  # keep track of the distance error
                
                if (distance_error >= -0.1):  # if the turtle is acceptably close to the destination
                    return True  # indicate that the destination was reached
                
        return False  # indicate that the distance has not been reached





    def rotation(self, ang_vel, theta_desired): 
        
        # Rotate the turtle to a desired angle
        
        if (self.msg_theta >= 0): # if the current rotation angle is greate than or equal to zero
                theta_travel = self.msg_theta - theta_desired  # calculate how far to rotate
                print('theta_travel = ', theta_travel)
                
                # Publish a command over the 'turtle1/cmd_vel' topic to rotate the turtle
                # The angular velocity published depends on how close the turtle is to the correct angle
                if (theta_travel > 0.1):
                        TurtlePainter.turtle_publish(self, 0.0, -3*ang_vel)
                elif (theta_travel > 0.001 and theta_travel < 0.1):
                        TurtlePainter.turtle_publish(self, 0.0, -0.05*ang_vel)
                elif (theta_travel < -0.1):
                        TurtlePainter.turtle_publish(self, 0.0, 3*ang_vel)
                elif (theta_travel > -0.1 and theta_travel < 0.0):
                        TurtlePainter.turtle_publish(self, 0.0, 0.05*ang_vel)
                else:  # if the turtle is already close enough to the correct angle
                        TurtlePainter.turtle_publish(self, 0.0, 0.0)  # stop the turtle
                        self.angle_found = True  # set the found angle state equal to true
                        return True  # indicate the angle was not reached
                
                return False  # indicate the angle was not reached
                        
                
        elif (self.msg_theta < 0): # if the current rotation angle is less than zero
                theta_travel = theta_desired - self.msg_theta  # calculate how far to rotate
                print('theta_travel = ', theta_travel)

                # Publish a command over the 'turtle1/cmd_vel' topic to rotate the turtle
                # The angular velocity published depends on how close the turtle is to the correct angle
                if (theta_travel > 0.1):
                        TurtlePainter.turtle_publish(self, 0.0, 3*ang_vel)
                elif (theta_travel > 0.001 and theta_travel < 0.1):
                        TurtlePainter.turtle_publish(self, 0.0, 0.05*ang_vel)
                elif (theta_travel < -0.1):
                        TurtlePainter.turtle_publish(self, 0.0, -3*ang_vel)
                elif (theta_travel > -0.1 and theta_travel < 0.0):
                        TurtlePainter.turtle_publish(self, 0.0, -0.05*ang_vel)
                else:
                        TurtlePainter.turtle_publish(self, 0.0, 0.0)
                        self.angle_found = True
                        return  True # indicate the angle was not reached
                 
                return False  # indicate the angle was not reached
                   
        
                                
    


def main(args=None):
    
    rclpy.init(args=args) # initialize node 
    
    turtle_painter = TurtlePainter(True) # True means pen is not activated (not drawing)
    
    ang_vel = 0.5  # set the turtle's angular velocity
    check = False  # variable for checking the state of the turtle
    get_to_x = 0.0 # x coordinate to travel to
    get_to_y = 0.0 # y coordiante to travel to
    
   
    #####################
    # Draw the Letter U #
    #####################
    
    # Stage One: Get to the starting point of the U
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 1.0  # x coordinate to travel to
        get_to_y = 8.0  # y coordiante to travel to
        rclpy.spin_once(turtle_painter) # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    # Stage Two: Draw the left side of the U
    check = False  
    turtle_painter.__init__(state=False)  # turn the pen on
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 1.0  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    #Stage Three: Draw the bottom of the U
    check = False
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 3.5  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position

    
    #Stage Four: Draw the right side of the U
    check = False
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 3.5  # x coordinate to travel to
        get_to_y = 8.0  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    

    #####################
    # Draw the Letter C #
    #####################
    
    #Stage Five: Get to the starting point of the C
    check = False
    turtle_painter.__init__(state=True) # turn the pen off
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 6.5  # x coordinate to travel to
        get_to_y = 8.0  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    

    # Stage Six: Turn to the angle pi and draw the top of the C
    turtle_painter.__init__(state=False)
    TurtlePainter.turtle_publish(turtle_painter, 0.0, 3.117) # rotate the turtle
    sleep(2) # pause while the turtle rotates
    TurtlePainter.turtle_publish(turtle_painter, 2.51, 0.0)  # drive the turtle forward
    sleep(2) # pause while the turtle drives forward
    
    
    #Stage Six: Draw the left side of the C
    check = False
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 4  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    #Stage Seven: Draw the bottom of the C
    check = False
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 6.5  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
        

    #####################
    # Draw the Letter A #
    #####################
    
    #Stage Eight: Get to the starting point of the A
    check = False
    turtle_painter.__init__(state=True)  # turn the pen off
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 7.0  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    #Stage Nine: Draw the left leg of the A
    check = False
    turtle_painter.__init__(state=False)  # turn the pen on
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 8.25  # x coordinate to travel to
        get_to_y = 8.0  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    #Stage Ten: Draw the right leg of the A
    check = False
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 9.5  # x coordinate to travel to
        get_to_y = 4.5  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    #Stage Eleven: Go back to finish the A
    check = False
    turtle_painter.__init__(state=True)  # turn the pen off
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 8.875  # x coordinate to travel to
        get_to_y = 6.25  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    

    # Stage Twelve: Turn to the angle pi and finish the A
    turtle_painter.__init__(state=False) # turn the pen onn
    TurtlePainter.turtle_publish(turtle_painter, 0.0, 1.22)  # rotate the turtle
    sleep(2)  # pause while the turtle rotates
    TurtlePainter.turtle_publish(turtle_painter, 1.2, 0.0)  # drive the turtle forward
    sleep(2)  # pause while the turtle drives forward
    
    
    # Stage Thirteen: Go back to the starting position
    check = False
    turtle_painter.__init__(state=True)  # turn pen off
    while (check == False):  # while turtle has not reached its destination
        get_to_x = 1.0  # x coordinate to travel to
        get_to_y = 8.0  # y coordiante to travel to
        rclpy.spin_once(turtle_painter)  # retrieve the turtle's current pose (coordiantes)
        check = TurtlePainter.pid_get_to_point(turtle_painter, get_to_x, get_to_y, ang_vel) # travel to desired position
    
    
    # Destroy ROS 2 nodes
    turtle_painter.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '__main__':
    main()
