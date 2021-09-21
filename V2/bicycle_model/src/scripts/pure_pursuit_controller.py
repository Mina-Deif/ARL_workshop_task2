#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import velocity_msg
     

# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m
Kpp = 3.0       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # in sec

msg_to_publish = velocity_msg()


def callback(message):
			 	                       																	
	msg_to_publish.delta = math.atan ( 2 * L * math.sin(message.alpha) / (message.ld) )		# calculating steering angle by pure pursuit = delta	in rad
	
	if msg_to_publish.delta > 0.5:
		msg_to_publish.delta = 0.5
	if msg_to_publish.delta < -0.5:
		msg_to_publish.delta = -0.5
	
	
	msg_to_publish.velocity=16.0   								# vehicle velocity in m/sec
	rospy.loginfo(msg_to_publish)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('pure_pursuit_controller')
sub = rospy.Subscriber('lateral_control_errors',lateral_errors_msg, callback)
pub = rospy.Publisher('steering_input', velocity_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
