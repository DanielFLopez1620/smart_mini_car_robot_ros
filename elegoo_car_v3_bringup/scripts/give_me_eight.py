#!/usr/bin/env python3

# ---------------- Imports related to ROS -------------------------------------
import rospy
from std_msgs.msg import Int32
    
# ------------------ Definition of functions ---------------------------------
def publisher():
    """
	Draw an eigth using /linear_move topic for Elegoo Car V3, by using the
	dc_motor_velocity.ino configuration.

	Make sure to be using ROSSerial for Arduino.
	"""
    try:
        pub_servo = rospy.Publisher('/servo', Int32, queue_size=10)
        pub_motors = rospy.Publisher('/linear_move', Int32, queue_size=10)
        rate_1 = rospy.Rate(1)
        rate_2 = rospy.Rate(2)
        rate_3 = rospy.Rate(0.2)
        rate_4 = rospy.Rate(1)
        
        rate_4.sleep()
        pub_motors.publish(1)
        rospy.loginfo("Going backwards . . .")
        rate_4.sleep()
        
        rate_1.sleep()
        pub_motors.publish(4)
        rospy.loginfo("Stopping . . .")
        rate_1.sleep()
        
        rate_3.sleep()
        pub_motors.publish(2)
        rospy.loginfo("Going left . . .")
        rate_3.sleep()
        
        rate_1.sleep()
        pub_motors.publish(4)
        rospy.loginfo("Stopping . . .")
        rate_1.sleep()
        
        rate_4.sleep()
        pub_motors.publish(0)
        rospy.loginfo("Going forward . . .")
        rate_4.sleep()
        
        rate_1.sleep()
        pub_motors.publish(4)
        rospy.loginfo("Stopping. . .")
        rate_1.sleep()
        
        rate_3.sleep()
        pub_motors.publish(3)
        rospy.loginfo("Going left . . .")
        rate_3.sleep()
        
        rate_1.sleep()
        pub_motors.publish(4)
        rospy.loginfo("Stopping . . .")
        rate_1.sleep()
        
        pub_servo.publish(90)
        rate_2.sleep()
        pub_servo.publish(45)
        rate_1.sleep()
        pub_servo.publish(135)
        rate_1.sleep()
        pub_servo.publish(90)
        rate_2.sleep()
        
    except rospy.ROSInterruptException:
        return


# -------------------------------- MAIN PROGRAM -------------------------------
if __name__ == '__main__':
    rospy.init_node('give_me_eigth')
    rospy.loginfo('give_me_eigth has been started!')
    publisher()
    rospy.loginfo('give_me_eigth has ended!')