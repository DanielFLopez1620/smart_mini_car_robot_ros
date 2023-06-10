# Smart Mini Robot Car with ROS

A collection of packages related to the usage of the Smart Robot Kit 3.0 from ELEGOO with ROS by using the real robot or the simulations and visualizations made.

**Keywords:** smart robot, rosserial, robot_description, simulation, gazebo, rviz, ROS, Arduino.

⠀⠀⠀⠀⠀⠀⠀⠀![view1](/images/elegoo_model_view1.jpg)
⠀⠀⠀⠀⠀⠀⠀⠀![view2](/images/elegoo_model_view2.jpg)
⠀⠀⠀⠀⠀⠀⠀⠀![solid_model](/images/elegoo_model_solidassem.png)

## License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Author: Daniel F López<br />
Maintainer: Daniel F López, dfelipe.lopez@gmail.com**

The collection of packages has been tested under [ROS] Noetic on Ubuntu 20.04.
This is a educational project, which purposes on demostrating the capabilities of ROS and [Arduino] for robotics, and explore other capabilities of ROS related with simulations and visualization. While also adding modifications for the robot to integrate other technologies and tools.

# Building and Installation

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd <your_workspace>/src
	git clone https://github.com/DanielFLopez1620/smart_mini_car_robot_ros.git 
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

# Packages

## elegoo_car_v3_bringup
Package oriented to configure and use the Arduino robot with ROSSerial for Arduino to control the motors, servo motor and ultrasonic sound by using publishers and subscribers. Also, you can find some examples for the car, and a custom /cmd_vel for operating, for example, with a joystick or the keybord (teleop_twist_keyboard). Explore the package [here](/elegoo_car_v3_bringup/README.md)

⠀⠀⠀⠀⠀⠀⠀⠀![cmd_vel_serial](/elegoo_car_v3_bringup/images/elegoo_car_bringup_cmd_vel.gif)

## elegoo_car_v3_description
Package related to the robot description for the robot, the CADs and designs were made by the author of the package using SolidWorks. There you can find the description made by the sw_exporter in a plain urdf and a xacro file for a shorter implementation in code. Explore the models and description [here](/elegoo_car_v3_description/README.md)

⠀⠀⠀⠀⠀⠀⠀⠀![simple_model](/elegoo_car_v3_description/images/elegoo_car_description_xacro_model.gif)

## elegoo_car_v3_gazebo
Package that uses the gazebo configurations implemented in the description to simulate the robot and its components like the servo motor, the DC motors and the ultrasonic sensor, which allows to move the car throught the world. Explore the simulation package [here](/elegoo_car_v3_gazebo/README.md)

⠀⠀⠀⠀⠀⠀⠀⠀![simple_room](/elegoo_car_v3_gazebo/images/elegoo_car_gazebo_simple_room.gif)

# Future Work

* Improve ultrasonic sensor definition for Gazebo.

* Add odometry to the real robot.

* Design a variation of the robot that does not need to be connected to the PC.

# Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/DanielFLopez1620/smart_mini_car_robot_ros/issues).