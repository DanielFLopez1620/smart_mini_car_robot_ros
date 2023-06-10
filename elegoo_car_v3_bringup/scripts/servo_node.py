#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Range
    
flag = False 
def pose_callback(data: Range):
    global flag 
    rospy.loginfo("Range: %s", data.range)
    if data.range < 0.3:
        flag = True
    else:
        flag = False
           
       
def subscriber():
    sub = rospy.Subscriber('/ultrasound', Range, pose_callback)
    
def publisher():
    global flag 
    while not rospy.is_shutdown():
        try:
            pub_1 = rospy.Publisher('/servo', Int32, queue_size=10)
            rate_1 = rospy.Rate(3)
            rate_2 = rospy.Rate(1)
            msg = Int32()
            if flag:
                msg.data = 90
                pub_1.publish(msg)
                rate_1.sleep()
                msg.data = 45
                pub_1.publish(msg)
                rate_2.sleep()
                msg.data = 135
                pub_1.publish(msg)
                rate_2.sleep()
                msg.data = 90
                pub_1.publish(msg)
                rate_1.sleep()
        except rospy.ROSInterruptException:
            return
        
if __name__ == '__main__':
    rospy.init_node('servo_node')
    rospy.loginfo('servo_node has been started!')
    subscriber()
    publisher()
    rospy.loginfo('servo_node has ended!')