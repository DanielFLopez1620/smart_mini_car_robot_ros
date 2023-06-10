#!/usr/bin/env python3

# ----------------- ROS related imports ---------------------------------
import rospy
from std_msgs.msg import Int32

# ------------------ Python related imports ----------------------------
import time
 
# ------------------ Definition of functions ---------------------------
def demo():
	"""
	Demostration of Elegoo Car V3 using /linear_move topic, for the
	dc_motor_velocity.ino configuration.

	Make sure to be using ROSSerial for Arduino.
	"""
	pub = rospy.Publisher('/linear_move', Int32, queue_size=10)

	rospy.init_node('demo_elegoo_car')
	
	time.sleep(2)
	rospy.loginfo("Going ahead . . .")
	pub.publish(0)
	time.sleep(1)
	rospy.loginfo("Turn . . .")
	pub.publish(2)
	time.sleep(1)
	rospy.loginfo("Turn . . .")
	pub.publish(3)
	time.sleep(1)
	rospy.loginfo("Backwards . . .")
	pub.publish(1)
	time.sleep(1)
	rospy.loginfo("Stop . . .")
	pub.publish(5)

# --------------------------- MAIN PROGRAM --------------------------
if __name__ == "__main__":
	demo()