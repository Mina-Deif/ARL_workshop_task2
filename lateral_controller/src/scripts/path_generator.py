#!/usr/bin/env python
import rospy
import math
import numpy as np
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import coordinates_msg
		  		  
####################### This node generates the vehicle path and calculates the target point and heading error #############################

Kpp = 0.1       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # (unit: sec)


msg_to_publish = lateral_errors_msg()


def callback(message):


	vehicle_x = message.x					# current x coordinates relative to rear axle
	vehicle_y = message.y					# current y coordinates relative to rear axle	
	vehicle_theta = message.theta				# current vehicle heading
																									
	v = message.v 						# vehicle speeed
                   						
	msg_to_publish.ld = Kpp * v				# calculating look ahead distance	
	
	flag = 0						# used for deteermining if look ahead distance intersected the path or not
	
	# arbitary values for initialization
	min_distance = 10000000.0
	x_of_min = 0.0
	y_of_min = 0.0
											

	# defining the path 	
	for x in np.arange(0 ,1000, 0.01):
		y=10*math.sin(0.2*x) 				# sinsuidal path 
		
		distance = math.sqrt((vehicle_x-x)**2+(vehicle_y-y)**2)	
		
		if (distance < min_distance):
			min_distance = distance
			x_of_min = x+5 									# 5 added for a smoother path
			y_of_min = y
		
		if ((distance > msg_to_publish.ld-0.05) and (distance < msg_to_publish.ld+0.05)):		# tolerance added due to float point inaccuricies
			target_x = x
			target_y = y
			flag = 1
		
		if (flag ==0 ):										# head for closest point on path if look ahead distance didn't intersect the path
			target_x = x_of_min
			target_y = y_of_min
		
		# future tuning: heading when no intersection occurs will be function of vehicle velocity for smoother motion


	# calculating (alpha) the heading error
	
	delta_x = (target_x-vehicle_x)
	delta_y = (target_y-vehicle_y)
	
	if delta_x ==0 and delta_y > 0:				# at 90 degrees
		slope = math.tan( math.pi/2 )
	elif delta_x ==0 and delta_y < 0:				# at 270 degrees
		slope = math.tan( math.pi*(3/2) )
	elif delta_x > 0 and delta_y == 0:				# at 0 degrees
		slope = 0
	elif delta_x < 0 and delta_y == 0:				# at 180 degrees
		slope = math.tan( math.pi )
	else:	
		slope = abs( delta_y / delta_x )
	
	angle = math.atan(slope)
	
	# setting angle to corresponding quadrant
	
	if delta_x < 0 and delta_y > 0:				# second quad
		angle = math.pi + angle	
	elif delta_x < 0 and delta_y < 0:				# third quad
		angle = math.pi - angle	
	elif delta_x > 0 and delta_y < 0:				# fourth quad
		angle = 2*math.pi - angle
	
	
	msg_to_publish.alpha =  angle - vehicle_theta 		# calculating heading error (unit: rad)	
    
	rospy.loginfo(msg_to_publish)
	rospy.loginfo(flag)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('path_generator')
sub = rospy.Subscriber('coordinates',coordinates_msg, callback)
pub = rospy.Publisher('lateral_control_errors', lateral_errors_msg, queue_size=10)
rate = rospy.Rate(1/time_step) #(unit: Hz)

rospy.spin()
