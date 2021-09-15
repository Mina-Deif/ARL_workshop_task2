#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import coordinates_msg


#array will be used to define the path points
#import numpy as np
#grid = np.array([[0,   0   ],
#		  [0.25, 0.2 ],
#		  [0.5 , 0.25],
#		  [0.75, 0.4 ],
#		  [1   , 0.55,],])
		  		  
# code section for determining the point on the path that is at a distance equal to the specified look ahead distance


# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m
Kpp = 3.0       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # in sec

msg_to_publish = lateral_errors_msg()

target_x = 20.0	# temporaraly for adjusting controller
target_y = 5.0		# temporaraly for adjusting controller

def callback(message):
	vehicle_x = message.x													# current x coordinates relative to rear axle
	vehicle_y = message.y													# current y coordinates relative to rear axle	
	vehicle_theta = message.theta												# current vehicle heading
																									
	v = message.v 														# vehicle speeed
			 	                       							
	msg_to_publish.ld = Kpp * v												# look ahead distance
	
	msg_to_publish.alpha = - math.atan( (target_y-vehicle_y) / (target_y-vehicle_y) ) - vehicle_theta + 3.1416		# calculating heading error		
    
	rospy.loginfo(msg_to_publish.alpha)
	rospy.loginfo(msg_to_publish.ld)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('path_generator')
sub = rospy.Subscriber('coordinates',coordinates_msg, callback)
pub = rospy.Publisher('lateral_control_errors', lateral_errors_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
