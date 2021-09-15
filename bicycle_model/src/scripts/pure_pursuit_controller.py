#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import lateral_errors_msg
from std_msgs.msg import Float64
     

# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m
Kpp = 3.0       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # in sec

msg_to_publish = Float64()


def callback(message):
			 	                       																	
	msg_to_publish = math.atan ( 2 * L * math.sin(message.alpha) / (message.ld) )	# calculating steering angle by pure pursuit = delta	
    
	rospy.loginfo(msg_to_publish)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('pure_pursuit_controller')
sub = rospy.Subscriber('lateral_control_errors',lateral_errors_msg, callback)
pub = rospy.Publisher('steering_input', Float64, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
