# Get started

Make sure you read and follow all the instructions here. Otherwise, I assure you **will** break the robot. **Ask**, if you do not understand anything mentioned below. Take you time to read and think!

## Deoxys

[Deoxys](https://ut-austin-rpl.github.io/deoxys-docs/html/) is a real-time controller library for Franka Emika Panda. To correctly deploy your policy on the real robot, you should read through the **whole** document. Especially, pay more attention to the [tutorial](https://ut-austin-rpl.github.io/deoxys-docs/html/tutorials/running_robots.html).

## Force Sensor

We have a SensONE 6-axis Force/Torque sensor mount to the end-effector of the robot.
1.	Read the official user [manual](http://www.jwcorporation.kr/wp-content/catal/BOTASYSTEMS.pdf) to know what this product is about, it is more than just a force/toque sensor!
2.	Read the official [FAQ](https://www.botasys.com/faq) to know what internal calibration and external calibration are. You will need to do external calibration before you use.

Use the code here!!! to calibrate the force sensor, try to find the offset and save it.
I have created a python module for reading the forces. You can skim through it although you do not have to understand everything, it is modified based on the official code example list below. 

some other useful links:
Official code [example](https://gitlab.com/botasys/python_interface/-/blob/main/examples/bota_serial_example.py) of using the sensor
Sensor structure [STL](https://gitlab.com/botasys/bota_driver/-/blob/master/rokubimini_description/meshes/BFT_SENS_M8/mounting.STL) and [URDF](https://gitlab.com/botasys/bota_driver/-/blob/master/rokubimini_description/urdf/BFT_SENS_SER_M8_robot.urdf.xacro) (you will need for calibrating the kinematic model)


## Operational Space Controller Example

I have provided an operational space + kp controller example!!!!! in this repo. Please read through every lines of code, if you do not understand why I have certain line there, **ask**.


---
Shuozhe Li (06/19/2023)
