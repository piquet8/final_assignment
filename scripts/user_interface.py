#! /usr/bin/env python

import rospy
import time
# import ros service
from std_srvs.srv import *

#function called when we have a new request
def set_new_state(req):
    print("\nTo control the robot in the environment you can choose among 4 different beahaviors : \n")
    print("1: move randomly \n")
    print("2: target position \n")
    print("3: walls following \n")
    print("4: stop \n")
    
    while True:
    	mode = input('Choose one of the 4 possible beahviours: ')
    	#check if the behaviour is among the 4 possible ones
    	if mode <1 or mode > 4:
    	        print('------------------------------------------------------')
    		print('Error: this value is not among the possibles behaviour')
    		print('------------------------------------------------------')			
    		continue
      	else:
        	state = mode
		break
     # setting the parameter
    rospy.set_param("state", state)
    #the state has been changed
    rospy.set_param('change_state',1)
    return []
    
def main():
    # initialising the node
    rospy.init_node('user_interface')
    # initialising the service server with argument the set_new_state Callback
    srv = rospy.Service('user_interface', Empty, set_new_state)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    	rate.sleep()
   	
	

if __name__ == '__main__':
    main()
