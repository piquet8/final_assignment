#! /usr/bin/env python

import rospy
import time
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

import math

# commonn publisher
pub_cmdvel = None
pub_mvbase = None
pub_goalid = None

# services for clients
srv_client_wall_follower_ = None
srv_client_main_node = None
srv_client_random_position = None

# actual robot position (global); initialised as a point!
position_ = Point()
# desired robot position (global); initialised as a point!
desired_position_ = Point()
# getting parameters from the launch file
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0

val_x=0
val_y=0

# Available states
state_desc_ = ['move randomly', 'target postion', 'walls following', 'stop']

# getting the state from the launch file
state_= rospy.get_param('state')

def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position
    
def target_distance():
    global position_
    x_des = rospy.get_param('des_pos_x')
    y_des = rospy.get_param('des_pos_y')
    x_pos = position_.x
    y_pos = position_.y
    distance = math.sqrt( ((y_pos-y_des)**2)+((x_pos-x_des)**2) )

    return distance

def publish_ag():

	global pub_mvbase
	msg_Action_Goal = MoveBaseActionGoal()
	msg_Action_Goal.goal.target_pose.header.frame_id="map";
	msg_Action_Goal.goal.target_pose.pose.orientation.w=1;

	#set in the position the desired ones
	msg_Action_Goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
	msg_Action_Goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');

	# publishing message goal
	pub_mvbase.publish(msg_Action_Goal)

	return[]

def change_state():
    global state_, state_desc_, y_val, x_val
    global srv_client_wall_follower_, srv_client_random_position,srv_client_main_node, pub_cmdvel, pub_mvbase
    state_ = rospy.get_param('state')
    log = "state changed: %s" % state_desc_[state_-1]
    rospy.loginfo(log)
    change_state=rospy.set_param('change_state',0)

    # 1 - move randomly
    if state_ == 1:
       		resp = srv_client_wall_follower_(False)
        	resp = srv_client_random_position()
		x_targ = rospy.get_param('des_pos_x')
    		y_targ = rospy.get_param('des_pos_y')
		print('New target: x:' + str(x_targ) + ' y:' + str(y_targ))
		publish_ag()
	

    # 2 - target position 
    if state_ == 2:
		resp = srv_client_wall_follower_(False)
            	while True:
                    print('Please, insert the desired target position between: [(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)] ')
            	    x_val=float(raw_input('x: '))
            	    y_val=float(raw_input('y: '))		
		# Control if the poision is one of the possible six
		    if x_val == -4 and (y_val == -3 or y_val == 2 or y_val == 7) :
	   		 rospy.set_param("des_pos_x", x_val)
	    		 rospy.set_param("des_pos_y", y_val)
	    		 break
		    elif x_val == 5 and (y_val == -3 or y_val == 1 or y_val == -7) :
	   		 rospy.set_param("des_pos_x", x_val)
	    		 rospy.set_param("des_pos_y", y_val)
	    		 break
		    else :
		        print('--------------------------------------------------------------------------------')
			print('Error: the coordinates you have chosen are not present among the 6 possible ones')
			print('--------------------------------------------------------------------------------')
			continue        
			
        # Publish the chosen position as a MoveBaseActionGoal
		x_targ = rospy.get_param('des_pos_x')
    		y_targ = rospy.get_param('des_pos_y')
		print('New target: x:' + str(x_targ) + ' y:' + str(y_targ))
		publish_ag()

    # walls following
    if state_ == 3:
    	    print('-------------------------------------------------------------------')
     	    print("\nThe robot will follow the walls until a new behaviour is chosen\n")
     	    print('-------------------------------------------------------------------')
	    resp = srv_client_wall_follower_(True)
	    resp = srv_client_main_node()
	   


    # 4 - stop
    if state_ == 4:
    	global position_
    	x_pos = position_.x
    	y_pos = position_.y
	resp = srv_client_wall_follower_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
	twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        pub_cmdvel.publish(twist_msg)
        resp = srv_client_main_node()
         

def set_new_state(req):
    print("\n Hi, to control the robot in the environment you can choose among 5 different beahaviors : \n")
    print("1: move randomly \n")
    print("2: target position \n")
    print("3: walls following \n")
    print("4: stop \n")
    
    while True:
    	mode = input('Choose one of the 5 possible beahviours: ')
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
    print("State chosen:" + str(mode))
    return []

def main():
    # initialisng the node
    rospy.init_node('main_node')
    # initialing the service server. As argument it takes the set_state Callback
    srv = rospy.Service('main_node', Empty, set_new_state)

    
    time.sleep(2)
    global position_, desired_position_, state_
    global srv_client_main_node, srv_client_wall_follower_, srv_client_random_position, pub_cmdvel, pub_mvbase, pub_goalid
    
    new_state=0
    # initialisng the master_node

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub_cmdvel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_mvbase = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1 )
    pub_goalid = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)


    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_client_random_position = rospy.ServiceProxy('/random_position',Empty)
    srv_client_main_node = rospy.ServiceProxy('/main_node', Empty)
    
    # initialize going to the point
    change_state() # to check the numb. state

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    # if a change occours in the state I have to update
    # the state machine. Otherwise I check the state (between 1 and 2)
    # Then I wait for the goal achievement before prompting the interface

    	new_state=rospy.get_param('change_state')
    	if new_state == 1:
    	    change_state()

    	else:
    	    state_=rospy.get_param('state')
            if state_==1 or state_==2:
    		    a=target_distance()
                    if a<0.5 :
    		            msg_goalid=GoalID()
    		            pub_goalid.publish(msg_goalid)
    		            resp = srv_client_main_node()
        rate.sleep()


if __name__ == '__main__':
    main()
