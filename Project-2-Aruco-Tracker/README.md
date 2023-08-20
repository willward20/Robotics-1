# Project 2: Aruco Tracker

## Summary
The goal of this project is to design and build a robot that navigates through an environment by driving towards ArUco markers. The control program `aruco_tracker.py` uses CV2 to capture images and detect ArUco marker patterns in the image. If a marker is detected, the program calculates the location of the center of the marker relative to the center of the image. If the markers is on the left side of the image, the robot drives forward and to the left (at a rotation angle proportional to how far the marker is from the center), and if the marker is on the right side of the image, the robot drives forward and to the right. If no marker is detected, the robot goes into "lost" mode, slowly turning on the spot to locate an ArUco marker. The forward left and forward right driving operations are added to the gpiozero PhaseEnableRobot class to provide more versatile driving.

**Demonstration:** [YouTube video](https://www.youtube.com/shorts/Ldh8a0Xqj6I)

### Materials List
* Raspberry Pi 3 Model B+: [CanaKit](https://www.canakit.com/raspberry-pi-3-model-b-plus-starter-kit.html)
* Pi Camera
* Motor Control Board: [Pololu Dual MC33926 Motor Driver for Raspberry Pi](https://www.pololu.com/product/2755)
* DC Motors
* Lithium Polymer Battery
* Rechargeable 5V USB Battery: [Miady HYD001](https://www.amazon.com/Miady-Portable-Charger-5000mAh-Lightweight/dp/B083VRD7CX)

### Software List
The `time` and `gpiozero` Python libraries were used to complete this project, and I programmed in the Thonny integrated development environment. From the `gpiozero` library, I used `LED`, and `PhaseEnableRobot` objects to program the motors.


