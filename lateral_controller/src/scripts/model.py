#!/usr/bin/env python
import rospy
import math
from bicycle_model.msg import velocity_msg
from bicycle_model.msg import coordinates_msg
     

# vehicle geometric parameters
L = 1.68        # Wheelbase (unit: m)
t = 1.1         # track width (unit: m)
lr = 0.68       # C.G to rear axle (unit: m) will be needed in odometery using encoders
rw = 0.3        # wheel radius (unit: m)


#time parameter
time_step = 1/100 # (unit: sec)

msg_to_publish = coordinates_msg()

# this model is built with the rear axle as the reference point

msg_to_publish.x = 0;
msg_to_publish.y = 0;
msg_to_publish.theta = 0;

def callback(message):
  
    v=message.velocity                          						# vehicle velocity (unit: m/sec)
    delta=message.delta                          						# steering angle  (unit: rad)
    
    msg_to_publish.theta += (((v * math.tan(delta))/L) * time_step)  			# vehicle heading (unit: rad)
    
  #	conditions for heading correction 
  #  if  msg_to_publish.theta > 2*math.pi:
  #  	 msg_to_publish.theta= msg_to_publish.theta - 2*math.pi
  #  if msg_to_publish.theta < 2*math.pi:
  #  	 msg_to_publish.theta= msg_to_publish.theta + 2*math.pi
    
    msg_to_publish.x += ((msg_to_publish.v * math.cos(msg_to_publish.theta)) * time_step)	# vehicle current x coordinate (unit: m)
    msg_to_publish.y += ((msg_to_publish.v * math.sin(msg_to_publish.theta)) * time_step)	# vehicle current y coordinate (unit: m)
    msg_to_publish.v = v	
    
    rospy.loginfo(msg_to_publish)
    rospy.loginfo("\n")
    
    
    pub.publish(msg_to_publish)
    


rospy.init_node('bicycle_model')
sub = rospy.Subscriber('steering_input',velocity_msg, callback)
pub = rospy.Publisher('coordinates', coordinates_msg, queue_size=10)
rate = rospy.Rate(1/time_step) #(unit: Hz)

rospy.spin()
