#!/usr/bin/env python
import rospy
import math
import numpy as np
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import coordinates_msg


#array will be used to define the path points
#import numpy as np
#path = np.array([[0,   0   ],
#		  [0.5 , 0.25],
#		  [0.75, 0.4 ],
#		  [1   , 0.55,],])
		  		  

# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m
Kpp = 0.3       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # in sec

msg_to_publish = lateral_errors_msg()


def callback(message):
	vehicle_x = message.x													# current x coordinates relative to rear axle
	vehicle_y = message.y													# current y coordinates relative to rear axle	
	vehicle_theta = message.theta												# current vehicle heading
																									
	v = message.v 														# vehicle speeed
                   							
	msg_to_publish.ld = Kpp * v												# look ahead distance
	
	for x in np.arange(0.0, 200.0, 0.005):
		y=math.sin(x)
		distance = math.sqrt((vehicle_x-x)**2+(vehicle_y-y)**2)	
		if ((distance > msg_to_publish.ld-0.5) and (distance > msg_to_publish.ld-0.5)):
				target_x = x
				target_y = y
						


				
	delta_x = (target_x-vehicle_x)
	delta_y = (target_y-vehicle_y)
	
	if delta_x ==0 and delta_y > 0:
		slope = math.pi/2
	elif delta_x ==0 and delta_y < 0:
		slope = math.pi*(3/2)
	elif delta_x > 0 and delta_y == 0:
		slope = 0
	elif delta_x < 0 and delta_y == 0:
		slope = math.pi
	else:	
		slope = delta_y / delta_x
	
	msg_to_publish.alpha = - math.atan(slope) - vehicle_theta + math.pi		# calculating heading error		
    
	rospy.loginfo(msg_to_publish.alpha)
	rospy.loginfo(msg_to_publish.ld)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('path_generator')
sub = rospy.Subscriber('coordinates',coordinates_msg, callback)
pub = rospy.Publisher('lateral_control_errors', lateral_errors_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
