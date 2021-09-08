#!/usr/bin/env python

#This function is a service that reply to a request with a random couple of coordinates whithin the 6 possibiles 
import rospy
import random
from std_srvs.srv import *
from final_assignment.srv import Random_position

#function called when we have a new request
def srv_callback(req):
	#random number between 1 and 6 that determines one of the possible pairs of coordiante
	numb = random.randint(1,6)
	#(-4,-3)
	if numb == 1:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", -3)
        #(-4,2)
	elif numb == 2:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", 2)
	#(-4,7)
	elif numb == 3:
		rospy.set_param("des_pos_x", -4)
		rospy.set_param("des_pos_y", 7)
	#(5,-7)
	elif numb == 4:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", -7)
	#(5,-3)
	elif numb == 5:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", -3)
	#(5,1)
	elif numb == 6:
		rospy.set_param("des_pos_x", 5)
		rospy.set_param("des_pos_y", 1)

        return []
	
#main node's function 
def main():
	#initialising the node
	rospy.init_node('random_position')
	# initialising the service server with argument srv_callback
	srv = rospy.Service('random_position', Empty, srv_callback)
	
	rate = rospy.Rate(20)
    	while not rospy.is_shutdown():
        	rate.sleep()

if __name__=='__main__':
	main()
