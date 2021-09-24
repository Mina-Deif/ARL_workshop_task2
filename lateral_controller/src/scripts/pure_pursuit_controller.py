#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import velocity_msg
     

# vehicle geometric parameters
L = 1.68        # Wheelbase (unit: m)


#time parameter
time_step = 1/100  # (unit: sec)

msg_to_publish = velocity_msg()


def callback(message):

	# correction for arctan values at second and third quads to give max steering angle
	 
	alpha = message.alpha
	if alpha > math.pi/2 and alpha <= math.pi:						# second quad
		alpha=	math.pi/2
	if alpha > math.pi and alpha < 3*math.pi/2:						# Third quad
		alpha=	-math.pi/2	
		 	                       																	
	msg_to_publish.delta = math.atan ( 2 * L * math.sin(alpha) / (message.ld) )		# calculating steering angle by pure pursuit equation   (unit: rad)
	
	if msg_to_publish.delta > 1.0:							# constrain maximum delta to 60 degrees (extreme for sake of trials)
		msg_to_publish.delta = 1.0
	if msg_to_publish.delta < -1.0:							# constrain minimum delta to 60 degrees
		msg_to_publish.delta = -1.0
	
	
	msg_to_publish.velocity=50.0   							# vehicle velocity (unit: m/sec)
	rospy.loginfo(msg_to_publish)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('pure_pursuit_controller')
sub = rospy.Subscriber('lateral_control_errors',lateral_errors_msg, callback)
pub = rospy.Publisher('steering_input', velocity_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # (unit: Hz)

rospy.spin()
