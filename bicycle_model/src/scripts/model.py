#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import velocity_msg
from bicycle_model.msg import coordinates_msg
     

# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m

#time parameter
time_step = 1/100  # in sec

msg_to_publish = coordinates_msg()

# this model is built with the rear axle as the reference point

def callback(message):
    wl=message.left_vel                          						# Rear left wheel angular velocity
    wr=message.right_vel                         						# Rear right wheel angular velocity
    delta=message.delta                          						# steering angle will be used later when transforming coordinates from rear axle to front axle
    
    msg_to_publish.v = (rw * wr + rw * wl)/2                          			# vehicle velocity 
    
    msg_to_publish.theta += ((v * math.tan(delta))/L) * time_step)  				# vehicle heading
    msg_to_publish.x += ((msg_to_publish.v * math.cos(msg_to_publish.theta)) * time_step)	# vehicle current x coordinate
    msg_to_publish.y += ((msg_to_publish.v * math.sin(msg_to_publish.theta)) * time_step)	# vehicle current y coordinate
    
    rospy.loginfo(msg_to_publish.x)
    rospy.loginfo(msg_to_publish.y)
    rospy.loginfo(msg_to_publish.theta)
    rospy.loginfo("\n")
    
    
    pub.publish(msg_to_publish)


rospy.init_node('bicycle_model')
sub = rospy.Subscriber('velocity_data',velocity_msg, callback)
pub = rospy.Publisher('coordinates', coordinates_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
