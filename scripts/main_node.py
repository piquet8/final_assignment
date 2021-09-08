#! /usr/bin/env python

# This node is the main one, in fact it handles the management of the robot within the simulation 
# using the other nodes and other methods implemented in it 

import rospy
import time
import math
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *
from final_assignment.srv import Random_position
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID


# publisher
pub_cmdvel = None
pub_mvbase = None
pub_goalid = None

# services for clients
srv_client_wall_follower_ = None
srv_client_user_interface = None
srv_client_random_position = None

# initialised actual robot position as a point
position_ = Point()
# initialised desired robot position as a point
desired_position_ = Point()
# getting parameters from the launch file
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0


# available states
state_desc_ = ['move randomly', 'target postion', 'walls following', 'stop']

# getting the state from the launch file
state_= rospy.get_param('state')

def clbk_odom(msg):
    global position_
    # position
    position_ = msg.pose.pose.position
    
# distance between the actual position of the robot and its target position
def target_distance():
    global position_
    x_des = rospy.get_param('des_pos_x')
    y_des = rospy.get_param('des_pos_y')
    x_pos = position_.x
    y_pos = position_.y
    distance = math.sqrt( ((y_pos-y_des)**2)+((x_pos-x_des)**2) ) 
    return distance

#communication with the move_base 
def publish_ag():
	global pub_mvbase
	#message of type MoveBaseActionGoal
	msg_Action_Goal = MoveBaseActionGoal()
	msg_Action_Goal.goal.target_pose.header.frame_id="map";
	msg_Action_Goal.goal.target_pose.pose.orientation.w=1;

	#set in the position the desired ones
	msg_Action_Goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
	msg_Action_Goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');

	# publishing message goal
	pub_mvbase.publish(msg_Action_Goal)

	return[]
	
# This function takes care of changing the status of the robot
def change_state():
    global state_, state_desc_, y_val, x_val, srv_client_wall_follower_, srv_client_random_position,srv_client_user_interface, pub_cmdvel, pub_mvbase
    #current state of the robot
    state_ = rospy.get_param('state')
    print("State chosen:%s" % state_desc_[state_-1])
    change_state=rospy.set_param('change_state',0)

    # 1 - move randomly
    if state_ == 1:
       		resp = srv_client_wall_follower_(False)
       		#call the service random_position for a random target position
        	resp = srv_client_random_position()
        	#set the new target position
		x_targ = rospy.get_param('des_pos_x')
    		y_targ = rospy.get_param('des_pos_y')
		print('New target: x:' + str(x_targ) + ' y:' + str(y_targ))
                # publish the chosen position as a MoveBaseActionGoal 
                publish_ag()
	

    # 2 - target position 
    if state_ == 2:
		resp = srv_client_wall_follower_(False)
            	while True:
                    # asks the user to enter a target position
                    print('Please, insert the desired target position between: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)] ')
            	    x_val=float(raw_input('x: '))
            	    y_val=float(raw_input('y: '))		
		    # control if the poision is one of the possible six
		    if x_val == -4 and (y_val == -3 or y_val == 2 or y_val == 7) :
	   		 rospy.set_param("des_pos_x", x_val)
	    		 rospy.set_param("des_pos_y", y_val)
	    		 break
		    elif x_val == 5 and (y_val == -3 or y_val == 1 or y_val == -7) :
	   		 rospy.set_param("des_pos_x", x_val)
	    		 rospy.set_param("des_pos_y", y_val)
	    		 break
		    else :
                        # if the chosen position there is not among the 6 possible ones		       
		        print('--------------------------------------------------------------------------------')
			print('Error: the coordinates you have chosen are not present among the 6 possible ones')
			print('--------------------------------------------------------------------------------')
			continue        
			
		x_targ = rospy.get_param('des_pos_x')
    		y_targ = rospy.get_param('des_pos_y')
		print('New target: x:' + str(x_targ) + ' y:' + str(y_targ))
		# publish the chosen position as a MoveBaseActionGoal
		publish_ag()

    # walls following
    if state_ == 3:
    	    print('-------------------------------------------------------------------')
     	    print("\nThe robot will follow the walls until a new behaviour is chosen\n")
     	    print('-------------------------------------------------------------------')
	    time.sleep(2)
	    # call the service wall follower
	    resp = srv_client_wall_follower_(True)
	    time.sleep(2)
	    # call the service user interface
	    resp = srv_client_user_interface()
	   


    # 4 - stop
    if state_ == 4:
    	global position_
    	x_pos = position_.x
    	y_pos = position_.y
	resp = srv_client_wall_follower_(False)
	# put equal to zero all the robot's velocities
        twist_msg = Twist()
        twist_msg.linear.x = 0
	twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        pub_cmdvel.publish(twist_msg)
        print('-------------------------------------------------------------------')
        print("\nThe robot is stationary in position x:"+str(x_pos)+" y:"+str(y_pos))
        print('-------------------------------------------------------------------')
        time.sleep(2)
        resp = srv_client_user_interface()
         

def main():
  
    global position_, desired_position_, state_, srv_client_user_interface, srv_client_wall_follower_, srv_client_random_position, pub_cmdvel, pub_mvbase, pub_goalid
    new_state=0
    # initialising the main_node
    rospy.init_node('main_node')
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub_cmdvel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_mvbase = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1 )
    pub_goalid = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    # associates the clients with the relative service
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_random_position = rospy.ServiceProxy('/random_position',Empty)
    srv_client_user_interface = rospy.ServiceProxy('/user_interface', Empty)
    
    change_state()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
	#if the status is 1 o 2 I have to wait until the robot reaches the target. 
    	#if there is a chande in status, I have to update the status of the machine.
    	new_state=rospy.get_param('change_state')
    	if new_state == 1:
    	    change_state()

    	else:
    	    state_=rospy.get_param('state')
            if state_==1 or state_==2:
    		    d=target_distance()
                    if d<0.5 :
    		            msg_goalid=GoalID()
    		            pub_goalid.publish(msg_goalid)
    		            print('\nTarget reached!\n')
    		            time.sleep(2)
    		            resp = srv_client_user_interface()
    rate.sleep()


if __name__ == '__main__':
    main()
