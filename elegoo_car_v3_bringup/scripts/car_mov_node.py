#!/usr/bin/env python3

# ---------------- Imports related to ROS -------------------------------------
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Range


flag = False 
# ----------------- FUNCTION DECLARATIONS -------------------------------------
def pose_callback(data: Range):
    global flag 
    if data.range < 0.3:
        flag = True
    else:
        flag = False

    
def subscriber():
    """
    Subscriber for ultrasonic sensor, validates sensor info to prevent collision
    """ 
    sub_ultrasound = rospy.Subscriber('/ultrasound', Range, pose_callback)
    
def publisher():
    con = True
    global flag 
    """
    Elegoo Car V3 Demo for using ROSSerial
    """ 
    while not rospy.is_shutdown():
        try:
            oriented_flag = [False, False]
            pub_servo = rospy.Publisher('/servo', Int32, queue_size=10)
            pub_motors = rospy.Publisher('/linear_move', Int32, queue_size=10)
            pub_motors.publish(0)
            if con:
                rospy.loginfo("Going ahead . . .")
                con = False
            rate_1 = rospy.Rate(2)
            rate_2 = rospy.Rate(1)
            rate_3 = rospy.Rate(0.4)
            rate_4 = rospy.Rate(0.25)
            
            msg = Int32()
            if flag:
                pub_motors.publish(4)
                msg.data = 90
                pub_servo.publish(msg)
                rate_1.sleep()
                msg.data = 45
                pub_servo.publish(msg)
                if flag:
                    oriented_flag[0] = True
                rate_2.sleep()
                msg.data = 135
                pub_servo.publish(msg)
                if flag:
                    oriented_flag[1] = True
                rate_2.sleep()
                msg.data = 90
                pub_servo.publish(msg)
                rate_1.sleep()
            if oriented_flag[0] and not oriented_flag[1]:
                pub_motors.publish(2)
                rate_3.sleep()
                rospy.loginfo("Going right. . .")
                con = True
            elif oriented_flag[1] and not oriented_flag[0]:
                pub_motors.publish(3)
                rate_3.sleep()
                rospy.loginfo("Going left . . .")
                con = True
            elif oriented_flag[0] and oriented_flag[1]:
                pub_motors.publish(1)
                rate_4.sleep()
                rospy.loginfo("Going backwards . . .")
                con = True
        except rospy.ROSInterruptException:
            msg.data = 90
            pub_servo.publish(msg)
            return
 
#--------------------------- MAIN PROGRAM DECLARATION -------------------------
if __name__ == '__main__':
    rospy.init_node('car_mov_node')
    rospy.loginfo('car_mov_node has been started!')
    subscriber()
    publisher()
    rospy.loginfo('car_mov_node has ended!')