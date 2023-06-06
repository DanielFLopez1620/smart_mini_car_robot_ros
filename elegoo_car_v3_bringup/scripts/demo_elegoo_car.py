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

	rospy.init_node('elegoo_demo')

	rate = rospy.Rate(0.5)
	
	time.sleep(5)
	rospy.loginfo("Avanzando")
	pub.publish(0)
	time.sleep(3)
	rospy.loginfo("Gira")
	pub.publish(2)
	time.sleep(3)
	rospy.loginfo("Gira")
	pub.publish(3)
	time.sleep(3)
	rospy.loginfo("Retrocede")
	pub.publish(1)
	time.sleep(3)
	rospy.loginfo("Detiene")
	pub.publish(5)

# --------------------------- MAIN PROGRAM --------------------------
if __name__ == "__main__":
	demo()