# RobotDog -  inexpensive open-source quadruped 

An easy quadruped made using python and C++. It uses:
<br/>
- 3D printed parts 
- 12 cheap SG90 servos (£1 each)
- 3mm plywood base 
- ESP32 and a 9-DOF Absolute Orientation IMU.
 </br>
It uses __pybullet__ and a URDF file for simulation and __ikpy__ for the inverse kinematic calculations.
<br/>
This is a remix of my final year project 
<a href="https://www.researchgate.net/project/Quadrupedal-Robotic-Platform-For-Research-on-Legged-Motion-Planning" target="_blank">Quadrupedal robotic Platform for research on legged motion</a>. It was originally built in a week during the Christmas holidays. (the UK was in lockdown, not much to do :confused:.) I am still looking through old files and fixed old code.
Find more of my work : <a href="http://jeromegraves.com/" target="_blank">jeromegraves.com</a>

## What's Here
There are two firmware Arduino files.

### MiniQuad Firmware
 The first is for a smaller quad (I'll find the CAD). This one has an IMU for detecting body rotation and 12 servos. The robot walks on the spot lifting two diagonal legs at a time. It then uses filtering and PID tuning of the rotational data from the IMU to modify the walk to stay balanced.

### MiniQuad2
MiniQuad 2 recives servo position information over UDP from a PC.
All the IK and control is done in python (__QuadGUI.py__). There is no IMU (rotation) sensor. This is just an experiment(failed) with IK in python. You can manipulate the position and rotation of the torso of the robot.
<br/>
There is also code to run a simple simulation of the quad using __pybullet__ and the __spotSimple.URDF__ file (__QuadSimpulation.py__).
<br/>
Run both __QuadGUI.py__ and __QuadSimulation.py__ to test without hardware.

### MiniQuad3 (Todo)
I didn't finish this remix as I ran out of free time.
The final version should be a combination of MiniQuad 1 + 2. 
It should have: 
-  Active balancing using IMU positional and rotation data.
-  Ik moved onto the ESP32 (so slow in python)
-  Simple control through phone (Bluetooth or webserver)
-  Set up ESP32 in AP wifi mode (generates its wifi network)
-  The robot will do a simple demo of translating and rotating its body and walking with active balancing.  

![Dog CAD Picture](images/dog-cad.png?raw=true "Title")

![Dog CAD Picture 2](images/dog-cad-2.gif?raw=true "Title")

## Parts
- 12X SG90 Digital Servo 
- ESP32 (not sure which one)
- 3D printer 
- IMU unit (not sure which one)

## ToDO
- Compute IK on ESP32.
- Add walk cycle.
- Add Bluetooth controller support.
- finish parts list.
- Test battery.
- Modify control to send four 3D vectors instead of 12 joint rotations.
- Add Bluetooth interface.
- ROS2 intergration?
