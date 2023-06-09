# Elegoo Car V3 Bring Up

## Overview

Package related to the configuration and bring up for the [ELEGOO] Smart Robot Car V3 with ROS and ROSserial for Arduino.

**Keywords:** smart robot, rosserial, Arduino, ROS.

### License

The source code is released under a [BSD 3-Clause license](/LICENSE).

**Author: Daniel F López<br />
Maintainer: Daniel F López, dfelipe.lopez@gmail.com**

The *elegoo_car_v3_bring_up* package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is a educational project, which purposes on demostrating the capabilities of ROS and [Arduino] for robotics to give those robots more capabilities and explore different applications with home made robots.


# Installation

## Installation from Packages

To use the codes available in this repository, make sure you have installed Arduino for Ubuntu 20.04, for that I recommend you to use some tutorials, like [Install Arduino IDE on Ubuntu 20.04 | Linux Op Sys](https://linuxopsys.com/topics/install-arduino-ide-on-ubuntu-20-04) or [Install the Arduino IDE | Canonical](https://ubuntu.com/tutorials/install-the-arduino-ide#5-thats-all-folks). After you have made the installation, make sure to configure the rosserial:

    sudo apt-get install ros-noetic-rosserial
	sudo apt-get install ros-noetic-rosserial-arduino
	cd ~<path_to_arduino_folder>/libreries
	rosrun rosserial_arduino make_libreries.py . # Make sure to run roscore first

Also, when connecting a Arduino Board or related, make sure to give permiission to the port, for example:

	sudo chmod a+rw /dev/ttyACM0

Finally, you can make sure you have installed all the dependencies with:

	sudo rosdep install --from-paths src
## Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (Middleware for robotics),
- [ROS Serial](https://wiki.ros.org/rosserial) (Protocol for wrapping ROS serialized message via serial port or network socket)
- [ROS Serial Arduino](https://wiki.ros.org/rosserial_arduino) (Rosserial client for Arduino)

## Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/ethz-asl/ros_best_practices.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

Also, you have to make sure you compile and run the .ino programs to the Arduino Board, to then run a node/application.

# Usage

The basic usage is to run the rosserial communication with a program running on the Arduino board, and then start to interact with the topics of the robot. Make sure to use the corresponding port, in the case of this repo:

	roscore
	rosrun rosserial_python serial_node.py /dev/ttyUSB0 # Or ACM0


# Nodes

## dc_motor_control

Serial node related to the move of the car forward or backwards according to the number sent. It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/base_mov`** ([std_msgs/int32])

	Case of movement: 0 is forward, 1 is backwards and otherwise is stop.

## dc_motor_on_off

Serial node related to move the car forward or backwards by toggling states. It must be uploaded to the Arduino and then execute the serial connection with ROS.

## Subscribed Topics

* **`/mov`** ([std_msgs/empy])

	When recieved, toggle the state of the car. If it is going forward, then it will be going backwards.

## dc_motor_velocity

Serial node related to the move of the car in 4 directions (forward, backwards, left or right). It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/linear_mov`** ([std_msgs/int32])

	Case of movement: 0 is forward, 1 is backwards, 2 is left, 3 is right, and otherwise is stop.

## elegoo_car_basic_usage

Serial node related to the move of the car in 4 directions (forward, backwards, left or right). It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/linear_mov`** ([std_msgs/Int32])

	Case of movement: 0 is forward, 1 is backwards, 2 is left, 3 is right, and otherwise is stop.

### Published Topics

* **`/ultrasound`** ([sensor_msgs/Range])

	The range identify by the ultrasonic sensor located in the front of the car.

## elegoo_car_cmd_vel

Serial node related to the command velocity for the car, that is compatible with joystick or the keyboard twist teleoperation. It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

	Topic related to the commands for moving and configuring the velocity of the robot.


## servo_control

Serial node related to control the servo motor that holds the ultrasonic sensor. It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/servo`** ([std_msgs/uint16])

	Indication of position to the servo, ideally between 0° and 180°.


## sonar_and_servo

Serial node related to using the servo motor and the ultrasonic sensor to detect obstacles. It must be uploaded to the Arduino and then execute the serial connection with ROS.

### Subscribed Topics

* **`/servo`** ([std_msgs/uint16])

	Indication of position to the servo, ideally between 0° and 180°.

### Published Topics

* **`/ultrasound`** ([sensor_msgs/Range])

	The range identify by the ultrasonic sensor located in the front of the car.

## car_move_node

Python node related to an simple obstacle avoidance showcase, it must use a serial node that has running /ultrasound, /servo and /linear_move at the same time, for example, **elegoo_car_basic_usage**.

### Subscribed Topics

* **`/ultrasound`** ([sensor_msgs/Range]) 

	The range identify by the ultrasonic sensor located in the front of the car.

### Published Topics

* **`/servo`** ([std_msgs/uint16])

	Indication of position to the servo, ideally between 0° and 180°, so it can decide the direction to turn.

* **`/linear_mov`** ([std_msgs/Int32])

	Case of movement: 0 is forward, 1 is backwards, 2 is left, 3 is right, and otherwise is stop.

## demo_elegoo_car

Python node related to a movement showcase, it must be used with a serial node that subscribes to /linear_move.

### Published Topics

* **`/linear_mov`** ([std_msgs/Int32])

	Case of movement: 0 is forward, 1 is backwards, 2 is left, 3 is right, and otherwise is stop.


## give_me_eigth

Another python node related to a movement showcase, it must be used with a serial node that subscribes to /linear_move.

### Published Topics

* **`/linear_mov`** ([std_msgs/Int32])

	Case of movement: 0 is forward, 1 is backwards, 2 is left, 3 is right, and otherwise is stop.

## servo_node

Python node related to test the ultrasonic sensor and the servo motor.

### Subscribed Topics

* **`/ultrasound`** ([sensor_msgs/Range]) 

	The range identify by the ultrasonic sensor located in the front of the car.

### Published Topics

* **`/servo`** ([std_msgs/uint16])

	Indication of position to the servo, ideally between 0° and 180°, so it can decide the direction to turn.

# Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/DanielFLopez1620/smart_mini_car_robot_ros/issues).


[ROS]: http://www.ros.org
[ELEGOO]: https://www.amazon.com/stores/page/E0F05684-D7AD-47CF-B08C-4084EBEE5BD3?ingress=2&visitId=16d40731-5924-4131-8f30-082353496e84&ref_=ast_bln
[Arduino]: https://www.arduino.cc/
[std_msgs/uint16]:(https://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt16.html)
[std_msgs/empy]:(https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html)
[std_msgs/int32]:(https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32.html)
[sensor_msgs/Range]:(https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html)
[geometry_msgs/Twist]:(https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)