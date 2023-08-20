# Project 1: Wall Bouncer

<img align="right" src="https://github.com/willward20/Robotics-1/blob/main/Gallery/ArucoFollower.PNG" width="300"/>

## Summary
The goal of this project is to design and build a robot that mimics popular Roombas (autonomous robots that vacuum the floor while driving through rooms and avoiding obstacles, like walls). The control program for this robot `wall_bouncer.py`, was written in Python using the gpiozero library. A Raspberry Pi is the central brain of the robot that runs the Python program. The robot has two DC motors that power the back wheels and are controlled by a Pololu Dual Driver Board connected to the Raspberry Pi. The driver board controls each wheel's Pulse Width Modulation (PWM) and direction to drive the robot forward and backward, turn left and right, and change the speed of each wheel.

Two ultrasonic sensors are attached on top of the robot to act as its eyes. Each sensor is constantly sending out ultrasonic frequency sound waves and receiving their transmitted signal. Based on the time it takes for the signal to return, the sensors determine the distance between the robot and an obstacle. The Raspberry Pi reads signals from the ultrasonic sensor, and if an obstacle is closer than a predeterimined distance to the robot, the program forces the robot to turn left until no objects are in its way. This section of the robot/program still needs some work. If the robot approaches an object at a small enough angle, the ultrasonic signals won't be transmitted back to the sensor, and the robot will run into the wall (see the wall bouncer video files). 

**Demonstration:** [YouTube video](https://www.youtube.com/watch?v=EKU1mnYIODc)

### Material List
* Raspberry Pi 3 Model B+: [CanaKit](https://www.canakit.com/raspberry-pi-3-model-b-plus-starter-kit.html)
* Resistors (220 Ohm, 10 kOhm, 330 Ohm, 470 Ohm)
* LEDs (Red, Green, and Yellow)
* Ultrasonic Sensors: [components101](https://components101.com/sensors/ultrasonic-sensor-working-pinout-datasheet)
* Motor Control Board: [Pololu Dual MC33926 Motor Driver for Raspberry Pi](https://www.pololu.com/product/2755)
* DC Motors
* Lithium Polymer Battery
* Rechargeable 5V USB Battery: [Miady HYD001](https://www.amazon.com/Miady-Portable-Charger-5000mAh-Lightweight/dp/B083VRD7CX)


### Software List
The `time` and `gpiozero` Python libraries were used to complete this project, and I programmed in the Thonny integrated development environment. From the `gpiozero` library, I used `LED`, `PWMLED`, `Button`, `DistanceSensor`, and `PhaseEnableRobot` objects to program the different sensors, motors, and circuit board elements.

### Usage
First, connect the Raspberry Pi onboard to its 5V power source and the motor control board to its Li-Po battery. A blue light will turn on on the motor control board. Run the python program `wall_bouncer.py`. The robot is ready to run when the green LED connected to the Pi starts slowly blinking on and off. This means that the robot is in "pause" mode. To make the robot start moving through the room, place it on the floor and press the push button on the breadboard. This puts the robot in "play" mode, and it will start driving. The robot is equipped with two ultrasonic distance sensors that help the robot avoid walls and other obstacles. 
A programmed time keeping function simulates the robot running out of battery. After the robot has been in "play mode" for 60 seconds, the yellow LED turns on, indicating that the "battery" is low and needs to be charged soon. After 90 seconds, the red LED starts flashing, indicating that the "battery" is very low and will be depleted soon. After another 10 seconds, the robot shuts itself down, as if the "battery" was completly depleted and the robot needed to be charged.
