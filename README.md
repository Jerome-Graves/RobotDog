# RobotDog - £20 open-source quadruped (not including 3D printer)
## About
An easy quadruped made using python and C++.
<br/>
It uses __pybullet__ and a URDF file for simulation and __ikpy__ for the inverse kinematic calculations.
<br/>
This is a remix of my final year project 
[Quadrupedal robotic Platform for research on legged motion](https://www.researchgate.net/project/Quadrupedal-Robotic-Platform-For-Research-on-Legged-Motion-Planning)
. It was origional built in a week during christmas holidays. (the UK was in lockdown, not much to do :confused:.) I am still looking though old files and fixed old code.

## Whats Here
###MiniQuad Firmware
There are 2 firmware arduino .ino files. The first is for a smaller quad (I'll find the CAD). This one has an IMU for detecting body rotation and 12 servos. The robot walk on the sopt lifting 2 diagonal legs at a time. It then uses filtering and PID tuning of the rotational data from the IMU to modify the walk to stay balanced.

### MiniQuad2
MiniQuad 2 recives servo position information over UDP from a PC.
All the IK and control is done in python (__QuadGUI.py__). There is no IMU (rotation) sensor. This is just an experiment(failed) with IK in python. You can manipulate the positon and rotation of the torso of the robot.
<br/>
There is also code to run a simple simulation of the quad using __pybullet__ and the __spotSimple.URDF__ file (__QuadSimpulation.py__).
<br/>
Run both __QuadGUI.py__ and __QuadSimulation.py__ to test without  hardware.

### MiniQuad3 (Todo)
I didn't really finish this remix as I ran out of free time.
The final verision shold be a combination of MiniQuad 1 + 2. 
It should have: 
-  Active balacing using IMU positional and rotaiton data.
-  Ik moved onto the ESP32 (so slow in python)
-  Simple contol though phone (blutooth or web server)
-  Set up ESP32 in AP wifi mode (generates it's own wifi network)
-  The robot will do a simple demo of translating and rotating its body and walking with active balancing.  

![Dog CAD Picture](images/dog-cad.png?raw=true "Title")

![Dog CAD Picture 2](images/dog-cad-2.gif?raw=true "Title")

## Parts
- 12X SG90 Digital Servo 
- ESP32 (not sure which one)
- 3D printer 
- IMU unit (not sure which one)

## ToDO
- Add ESP code to repo.
- Compute IK on ESP (very slow in python).
- Add walk cycles.
- Add bluetooth controller support.
- finish parts list
- test battery
