#!/usr/bin/env python
import rospy
from bicycle_model.msg import velocity_msg

def publisher():
    pub = rospy.Publisher('velocity_data', velocity_msg, queue_size=10)
    
    rate = rospy.Rate(100) # Hz
    msg_to_publish = velocity_msg()
    
# This function publishes the angular velocities of the two rear wheels 

    while not rospy.is_shutdown():
        msg_to_publish.left_vel  = 0.0
        msg_to_publish.right_vel  = 1.0
        msg_to_publish.delta  = 3.0    #added for future optimization whaen transforming coordinates from rear axle refernce to front
        pub.publish(msg_to_publish)
        rospy.loginfo(msg_to_publish.left_vel)
        rospy.loginfo(msg_to_publish.right_vel)
        rospy.loginfo(msg_to_publish.delta)
        
        rate.sleep()

if __name__ == '__main__':
   rospy.init_node('velocity_steering_driver')
   publisher()
   
