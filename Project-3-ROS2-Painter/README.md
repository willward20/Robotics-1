# Project - ROS2 Managed Robots
UCA Robotics 1 Final Project. In this project, we control a simulated turtle bot to write **UCA** on a simulation canvas. In the mean 
time, a physical hardware "Hexa" robot mimics (part of) the turtle's behavior. All the tasks are managed by ROS2. Demonstration videos:
* [turtle_painter](https://youtu.be/S0M757739Hs)
* [hexar_driver](https://youtu.be/7ao2tx5IoJk) (the laptop screen shows the turtle_painter progress)

<img src="https://github.com/willward20/Robotics-1/blob/main/Gallery/turtlesim_example.png" width="300">   <img src="https://github.com/willward20/Robotics-1/blob/main/Gallery/Project3_Hexar.png" width="600">

## Summary
This project contains two ROS 2 packages used to control the virtual turtle in the turtlesim_node and a real "hexar" robot with 
motor controlled wheels. The turtle spells out "UCA," and the hexar robot just spells the "U" in sync with the turtle.
The `turtle_painter.py` program creates a turtle_painter node that subscribes to the turtle's position topic 
(`/turtle1/pose`) and publishes to the turtle's velocity topic (`/turtle1/cmd_vel`). The turtle_painter node is also 
a client of the turtle's SetPen service to turn the turtle's pen on or off. In main, the turtle's movements are 
divided into stages. Each stage moves the turtle to a pre-defined point on the xy plane by having the turtle rotate 
to face the point and then drive towards point. At the beginning of each stage, the callback function 
of the subcriber is spun once to store the turtle's current position and angle. 
Then, each stage sends the destination coordinates to a get_to_point function. This function determines how far away the 
turtle is from where it needs to go, linearly and angularly. First, the turtle is rotated to face the destination by publishing 
angular z velocities. When the angle is within a certain margin of error, the function moves on to driving the turtle
forward, all the while calculating the turtle's distance from the point. When the distance to the destination is within a certain error margin, the program sets a new destination for the turtle. The last stage is to drive the turtle back to the starting point of the U using a PD (proportional derivative) controller to minimize the distance between the turtle and the destintation.

Meanwhile, the `bringup_hexar.py` program also creates a node that subscribes to the `turtle1/cmd_vel` topic, although the values of the linear.x velocity and angular.z velocity do not matter to the robot. The hexar robot is so much faster 
than the turtle that you would need a very large driving space to translate the turtle velocites directly into hexar motor 
velocities. Instead, the hexar robot node only checks if each velocity is positve or zero. The hexar main program is divided 
into stages just like the turtle painter's program. When the turtle_paint node is publishing linear x velocites, hexar 
robot drives forward for a pre-determined amount of time. These were pre-determined through trial and error to get the robot
to turn to the right angle and drive forward the right distance. After the motors spin for a set amount of time, the program waits for the turtle_painter node to finish publishing linear x velocities and start publishing angular z velocites. The wait time is important because the turtle moves slower than the real robot and needs time to catch up. This cycle of driving and waiting on the turtle repeats until the program finishes. In its current state, the hexar robot only draws the U, but having it draw the rest of the letters would just be a matter of writing new stages and fine tuning the driving times. 
 

### Material List
* Raspberry Pi 3 Model B+: [CanaKit](https://www.canakit.com/raspberry-pi-3-model-b-plus-starter-kit.html)
* Motor Control Board: [Pololu Dual MC33926 Motor Driver for Raspberry Pi](https://www.pololu.com/product/2755)
* DC Motors
* Robot chassis and wheels
* Lithium Polymer Battery (Gold EDT Lipo Plus 7.4V 50C 5200mAh)
* Rechargeable 5V USB Battery: [Miady HYD001](https://www.amazon.com/Miady-Portable-Charger-5000mAh-Lightweight/dp/B083VRD7CX)

### Software List
* *Python Libraries*
  * `turtlesim.srv` (imported `SetPen`)
  * `rclpy`
  * `rclpy.node` (imported `Node`)
  * `geometry_msgs.msg` (imported `Twist`)
  * `turtlesim_msg` (imported `Pose`)
  * `math` (imported `atan`)
  * `time` (imported `sleep`)
  * `gpiozero` (imported `LED` and `PhaseEnableRobot`)
* *Other Software*
  * Ubuntu 20.04 (flashed to the RaspberryPi)
  * ROS 2 Galactic Geochelone (installed on RPi and Ubuntu laptop)
  * Configure the GPIO pins and SPI communications under Ubuntu Server. Use this [guide](https://github.com/linzhangUCA/robotics1-2021/wiki/Install-Ubuntu-Server-20.04-and-GPIO-Configuration)




### Usage
1. Run `ros2 run turtlesim turtlesim_node` on the Ubuntu laptop to bring up the turtle simulator. 
2. Next, open a terminal on the RPi and `cd` into the project workspace directory, build the project with `colcon build --symlink-install`, and source it with `. install/local_setup.bash`. 
3. Start the hexar robot on the Pi with `ros2 run hexar_driver bringup_hexar`. 
4. In a terminal on the laptop, `cd` into the project workspace directory and build and source like before. 
5. Start the `turtle_painter.py` program on the laptop with `ros2 run turtle_painter paint_uca`. 

Now, the turtle_painter node is subscribed to the turtle's position topic `/turtle1/pose`, and the program publishes velocity commands over the 
`/turtle1/cmd_vel` topic to turn and drive the turtle towards the pre-determined points. The real hexar robot
also subscribes to the cmd_vel topic and will drive with the turtle. If it reaches the destination before the turtle,
the real robot will pause until the turtle catches up. 

To have the robots spell different letters, simply change the destination points in the main body of the `turtle_painter.py`
program and change the sleep times on each stage of `bringup_hexar.py` program so the real robot will drive to the same
points as the turtle. To see the trail the hexar robot follows, attach a marker to the back and place the robot on 
surface you wish it to draw upon. 
