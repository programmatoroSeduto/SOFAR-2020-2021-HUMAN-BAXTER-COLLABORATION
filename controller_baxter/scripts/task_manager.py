#! /usr/bin/env python

## @package controller_baxter
#
#  \file task_manager.py
#  \brief This file allows the baxter to decide what to do
#
#  \authors Francesco Ganci, Zoe Betta, Lorenzo Causa, Federico Zecchi
#  \version 1.0
#  \date 12/06/2021
#  \details
#      # subscribe: unity_tf o tf depending on settings
#
#  Subscribes to: <BR>
#	 /unity_tf
#    /tf
#
#  Services: <BR>
#    controller_server
#    baxter_at_home_server
#
#  Description: <BR>
#    This node implements the choice of the actions that the two arms need to complete.
#    In order to do so we  decided to force the right arm to only pick blocks from 
#    the right side of the table and putting them at the center of the table. If
#    there is already a block in the center then the right arm waits and does nothing.
#    If there is not a block it decides to pick the block that is not too close
#    to the hand of the operator that is accessible and betwee those the furthest away
#    from the hands. Similar for the left arm, it picks blocks from the center or from the
#    left side of the table and it puts them in the goal box. The rules to 
#    decide which box to pick are similar to the ones of the right arm: it excludes
#    a priori all the blocks too close to the human operator and all the blocks non 
#    accessible; then it picks a block in the center, if present, or it looks for the
#    block furthest away from the hand of the human operator. These procedures
#    are repeated until all blocks are in the goal box.

import rospy
from controller_baxter.srv import at_home, at_homeRequest, at_homeResponse
from controller_baxter.srv import command, commandRequest, commandResponse
from human_baxter_collaboration.msg import UnityTf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Transform, Vector3, Quaternion

import math
import sys

import controller_baxter.sim_infos as info

## --------------------------- GLOBAL VARIABLES

# position of the hand of the operator
frame_human_hand_right = Pose()
frame_human_hand_left = Pose()

# position of all red blocks
frame_block_red = { "A" : Pose(), "L" : Pose(), "B" : Pose(), "H" : Pose(), "D" : Pose(), "F" : Pose() }

# position of all blue blocks
frame_block_blue = { "E" : Pose(), "C" : Pose(), "I" : Pose(), "G" : Pose(), "M" : Pose() }

# frame of the blue goal
frame_goal_blue = Pose()

# frame of the red goal
frame_goal_red = Pose()

# position of the table
frame_table = Pose()
frame_table.position.x = 0.5
frame_table.position.y = 0
frame_table.position.z = 0

# how many times the movement failed
miss = 0

# is the center free?
block_at_center = None

## --------------------------- TOPIC AND SERVER

# counter of updates
update_counter = 0

# service controller_server"
srv_controller = None

# service "baxter_at_home_server"
srv_baxter_home = None

# topic "unity_tf"
msg_unity_tf = None

##
#	\brief end of update log
#	\param None
#	\return : None
# 	
#	This function updates the log with the positon of the frames of the blue 
#   and red blocks. It also saves the iteration number.
def update_log( ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# check and counter of updates
	update_counter = update_counter + 1
	
	if( update_counter % info.log_freq == 0 ):
		rospy.loginfo( " [task manager] update no. %d", update_counter )
		rospy.loginfo( " [task manager]     %s len:%d", "frame_block_blue", len( frame_block_blue ) )
		rospy.loginfo( " [task manager]     %s len:%d", "frame_block_red", len( frame_block_red ) )
	if( update_counter % info.log_freq_extended == 0 ):
		rospy.loginfo( " [task manager] update no. %d EXTENDED LOG", update_counter )
		rospy.loginfo( " [task manager] frame_block_red:\n========\n %s\n========\n", str( frame_block_red ) )
		rospy.loginfo( " [task manager] frame_block_blue:\n========\n %s\n========\n", str( frame_block_blue ) )

##
#	\brief updating the position of red blocks
#	\param None
#	\return : None
# 	
#	This function updates the positon of the red blocks deleting the ones that
#   have alreaby been put in the goal from the list of available blocks. 
#   By doing so we are able to quickly check if evrything is in the correct place
def update_red_blocks( ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# deleting red blocks already in the goal
	labels = list(frame_block_red.keys())
	for elem in labels:
		if block_in_red_goal( elem ):
			rospy.loginfo( " [task manager] -> RED block %s in goal", elem )
			del frame_block_red[ elem ]
			if len( frame_block_red ) == 0 :
				rospy.loginfo( " [task manager] human job done." )

##
#	\brief trasforming from TransformStamped to Pose
#	\param Data : in format TransformStamped, the data I want to obtain in Pose format
#	\return : to_return the data in Pose format
# 	
#	This function transforms the input data from TransformedStamped to Pose
#   by assigning the respective members.
def tranformStamped_to_pose( data ):
	to_return = Pose()
	to_return.orientation = data.transforms[0].transform.rotation
	to_return.position.x = data.transforms[0].transform.translation.x
	to_return.position.y = data.transforms[0].transform.translation.y
	to_return.position.z = data.transforms[0].transform.translation.z
	
	return to_return

##
#	\brief callback topic unity
#	\param Data : data received from unity
#	\return : None
# 	
#	This function saves the information about each block received by the subscriber 
#   in the correspective global variable.
def callback_unity( data ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# label blocks
	labels_red = frame_block_red.keys()
	labels_blue = frame_block_blue.keys()
	
	for i in range( len(data.frames) ):
		# label frame unity
		name = data.frames[i].header.frame_id
		
		
		if   ( name in frame_block_blue ):
			frame_block_blue[ name ] = data.frames[i].pose
			
		elif ( name in frame_block_red ):
			frame_block_red[ name ] = data.frames[i].pose
				
		elif ( name == info.name_human_hand_right ):
			frame_human_hand = data.frames[i].pose
		
		elif ( name == info.name_human_hand_left ):
			frame_human_hand = data.frames[i].pose
			
		elif ( name == info.name_goal_blue ):
			frame_goal_blue = data.frames[i].pose
			
		elif ( name == info.name_goal_red ):
			frame_goal_red = data.frames[i].pose
			
		else:
			# do nothing
			pass
	
	# delete the red blocks already in the goal
	update_red_blocks( )
	
	# log di fine aggiornamento
	update_log( )

##
#	\brief callback topic tf
#	\param Data : data received from tf
#	\return : None
# 	
#	This function receives the information, divides them and saves them 
#   in the corresponding global variable.
def callback_tf( data ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# block labels
	labels_red = frame_block_red.keys()
	labels_blue = frame_block_blue.keys()
	
	name = data.transforms[0].child_frame_id;
	
	if   ( name in frame_block_blue ):
		frame_block_blue[ name ] = tranformStamped_to_pose( data )
		
	elif ( name in frame_block_red ):
		frame_block_red[ name ] = tranformStamped_to_pose( data )
		if block_in_red_goal( name ):
			rospy.loginfo( " [task manager] -> RED block %s in goal", name )
			del frame_block_red[ name ]
			if len( frame_block_red ) == 0 :
				rospy.loginfo( " [task manager] human job done." )
		
	elif ( name == info.name_human_hand_right ):
		frame_human_hand = tranformStamped_to_pose( data )
	
	elif ( name == info.name_human_hand_left ):
		frame_human_hand = tranformStamped_to_pose( data )
		
	elif ( name == info.name_goal_blue ):
		frame_goal_blue = tranformStamped_to_pose( data )
		
	elif ( name == info.name_goal_red ):
		frame_goal_red = tranformStamped_to_pose( data )
		
	else:
		# do nothing
		pass
	
	# end of update log
	if info.use_verbose:
		update_log( )
	else:
		update_counter = update_counter + 1

## --------------------------- FUNCTIONS

##
#	\brief distance between two poses
#	\param pose_A : variable of type Pose
#   \param pose_B : variable of type Pose 
#	\return : the distance
# 	
#	This function calculates the euclidean distance bewtween two poses
#   without considering the z coordinate. It calculates the euclidean distance
#   considering the objects on the plane.
def distance_between( pose_A, pose_B ):
	A = [ pose_A.position.y, pose_A.position.x ]
	B = [ pose_B.position.y, pose_B.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) A[%f, %f] B[%f, %f]", "distance_between", A[0], A[1], B[0], B[1] )
	
	return math.sqrt( (B[0]-A[0])**2 + (B[1]-A[1])**2 )

##
#	\brief checks whether the red block is in the red goal
#	\param elem : a string representing the block I want to check
#	\return : True if the block is in the goal, false otherwise
# 	
#	This function controls if a given red block is inside the red box that 
#   is the goal. It checks if the x and y coordinate of the center of the
#   frame of the object are distant more than half the size of the target box.
def block_in_red_goal( elem ):
	global frame_block_red, frame_goal_red
	
	# position of the block and the goal
	P = [ frame_block_red[elem].position.y, frame_block_red[elem].position.x ]
	O = [ frame_goal_red.position.y, frame_goal_red.position.x ]
	
	if info.use_verbose:
		#rospy.loginfo( " [task manager] (%s) %s[%f, %f] 0[%f, %f] x in [%f, %f] y in [%f, %f]", "block_in_red_goal", elem, P[0], P[1], O[0], O[1], O[0] - info.sz_goal[0]/2, O[0] + info.sz_goal[0]/2, O[1] - info.sz_goal[2]/2, O[1] + info.sz_goal[2]/2 )
		pass
	
	# returns true only if the block is already inside its goal
	if( P[0] < (O[0] - info.sz_goal[0]/2 ) or P[0] > (O[0] + info.sz_goal[0]/2 ) ):
		return False
	if( P[1] < (O[1] - info.sz_goal[2]/2 ) or P[1] > (O[1] + info.sz_goal[2]/2 ) ):
		return False
	
	return True

##
#	\brief checks whether the blue block is in the blue goal
#	\param elem : a string representing the block I want to check
#	\return : True if the block is in the goal, false otherwise
# 	
#	This function controls if a given blue block is inside blue box that 
#   is the goal. It checks if the x and y coordinate of the center of the
#   frame of the object are distant more than half the size of the target box.
def block_in_blue_goal( elem ):
	global frame_block_blue, frame_goal_blue

	# position of the block and the goal
	P = [ frame_block_blue[elem].position.y, frame_block_blue[elem].position.x ]
	O = [ frame_goal_blue.position.y, frame_goal_blue.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] 0[%f, %f] x in [%f, %f] y in [%f, %f]", "block_in_blue_goal", elem, P[0], P[1], O[0], O[1], O[0] - info.sz_goal[0]/2, O[0] + info.sz_goal[0]/2, O[1] - info.sz_goal[2]/2, O[1] + info.sz_goal[2]/2 )
	
	# returns true only if the block is already inside its goal
	if( P[0] < (O[0] - info.sz_goal[0]/2 ) or P[0] > (O[0] + info.sz_goal[0]/2 ) ):
		return False
	if( P[1] < (O[1] - info.sz_goal[2]/2 ) or P[1] > (O[1] + info.sz_goal[2]/2 ) ):
		return False
	
	return True

##
#	\brief checks whether a block is on the right side of the table
#	\param elem : ( 'name', Pose ) saves the name of the object and its Pose
#	\return : True if the block is in the right side, false otherwise
# 	
#	This function controls if a given block is in the right side of the table
#   by checking if the x and y coordinates of the center of the frame are
#   in the right side of the table, the side with negative coordinates.
def block_on_right_table( elem ):
	global frame_table
	
	# position of the block and the table
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_table.position.y, frame_table.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | x NOT in [%f, %f] ? | y NOT in [%f, %f] ? | test: (%s, %s)", "block_on_right_table", elem[0], P[0], P[1], O[0], O[1], O[0], O[0] + info.sz_table[0]/2, O[1] - info.sz_table[2]/2, O[1] + info.sz_table[2]/2, str( P[0] > O[0] or P[0] < (O[0] - info.sz_table[0]/2) ), str( P[1] < (O[1] - info.sz_table[2]/2) or P[1] > (O[1] + info.sz_table[2]/2) ) )
	
	# returns true only if the block is on the right side of the table
	if( P[0] > O[0] or P[0] < (O[0] - info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True

##
#	\brief checks whether a block is on the left side of the table
#	\param elem : ( 'name', Pose ) saves the name of the object and its Pose
#	\return : True if the block is in the right side, false otherwise
# 	
#	This function controls if a given block is in the left side of the table
#   by checking if the x and y coordinates of the center of the frame are
#   in the left side of the table, the side with positive coordinates.
def block_on_left_table( elem ):
	global frame_table
	
	# position of the block and the table
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_table.position.y, frame_table.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | x NOT in [%f, %f] ? | y NOT in [%f, %f] ? | test: (%s %s)", "block_on_left_table", elem[0], P[0], P[1], O[0], O[1], (O[0] - info.sz_table[0]/2), O[0], O[1] - info.sz_table[2]/2, O[1] + info.sz_table[2]/2, str( P[0] < O[0] or P[0] > (O[0] + info.sz_table[0]/2) ), str( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ) )
	
	# returns true only if the block is on the left side of the table
	if( P[0] < O[0] or P[0] > (O[0] + info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True

##
#	\brief get the closest position of the hand
#	\param elem : Pose the position with respect to which I want to find the closes had
#	\return : the frame of the closest hand
# 	
#	This function checks the distance of the position given as input from the
#   left and right hand of the operator, it returns the frame of the hand with
#   the smaller distance
def get_nearest_hand_pose( elem ):
	global frame_human_hand_right, frame_human_hand_left
	frame_human_hand = None
	if( distance_between( elem, frame_human_hand_left ) > distance_between( elem, frame_human_hand_right ) ):
		return frame_human_hand_right
	else:
		return frame_human_hand_left

# signal if the block is too close to the operator
#  elem : ( 'name', Pose ) 
##
#	\brief signal if the block is too close to the operator
#	\param elem : ( 'name', Pose ) saves the name of the object and its Pose
#	\return : true if the hand is further than a given threshold, false otherwise
# 	
#	This function first finds the nearest hand then calculates the euclidean
#   distance of the x and y coordiantes of the frame of the block and the 
#   frame of the hand. It doesn't consider the distance along z. It returns true 
#   if the disance is greater than a given threshold, false otherwise
def block_not_near_to_hand( elem ):
	# look for the closest hand to the block
	frame_human_hand = get_nearest_hand_pose( elem[1] )
	
	# calculate the distance between the hand and the block frame
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_human_hand.position.y, frame_human_hand.position.x ]
	dist = math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | dist: %f | limit: %f | result: %s", "block_not_near_to_hand", elem[0], P[0], P[1], O[0], O[1], dist, info.sz_hand, str( dist >= info.sz_hand ) )
	
	return True if ( dist >= info.sz_hand ) else False

##
#	\brief checks if the block is accessible
#	\param elem : ( 'name', Pose ) saves the name of the object and its Pose
#	\return : true if the hand is further than a given threshold, false otherwise
# 	
#	This function checks the distance between the block given as input and 
#   all of the red blocks. The distance are calculated along x, y and z.
#   If the distances are less than a given threshold then the block is not 
#   accessible and it returns false. As soon as the block is not accessible 
#   with respect to another red block the function returns false. Only if 
#   the block is accessible with respect to all redblocks then the function
#   returns true.
def block_is_accessible( elem_b ):
	global frame_block_red

	for elem_r in frame_block_red.values():
		dx = abs( elem_b[1].position.y - elem_r.position.y )
		dz = abs( elem_b[1].position.x - elem_r.position.x )
		dy = elem_b[1].position.z - elem_r.position.z 
		
		if info.use_verbose:
			rospy.loginfo( " [task manager] (%s) dx=%f | dy=%f | dz=%f | sovrapposizione: %s", "block_is_accessible", dx, dy, dz, str(dx < info.sz_cube and dz < info.sz_cube and dy < 0) )
		# returns true only if the block is not superimposed to another one
		if (dx < info.sz_cube and dz < info.sz_cube and dy < 0):
			return False

	return True

##
#	\brief calculate the distance between the hand and a block
#	\param elem : ( 'name', Pose ) saves the name of the object and its Pose
#	\return : the euclidean distance between the block and the closest hand
# 	
#	This function first finds the closest hand to the block given as input
#   and then calculates the euclidean distance between the x and y coordinates
#   of the block and the nearest hand
def dist_block_hand( elem ):
	# look for the closest hand to the block
	frame_human_hand = get_nearest_hand_pose( elem[1] )
	
	#  calculate the distance bewìtween the two frame origin
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_human_hand.position.y, frame_human_hand.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | dist: %f", "dist_block_hand", elem[0], P[0], P[1], O[0], O[1], math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 ) )
	
	return math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )

##
#	\brief check if the block is in the center of the table
#	\param elem : Pose, the position of the block I want to check
#	\return : true if the block is in the center
# 	
#	This function finds the position of the center of the table then 
#   calculates the distance between the two elements. It returns true if
#   the distance is below a threshold, false otherwise
def check_block_is_at_center( elem ):
	O = Pose()
	O.position.x = info.sz_table[0]/2
	O.position.y = 0
	O.position.z = 0
	
	# calculate the distance between the block and the center
	dist = distance_between( elem, O )
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) [%f, %f, %f] | distance: %f | limit: %f | answer: %s", "check_block_is_at_center", O.position.x, O.position.y, O.position.z, dist, info.center_dispacement, str( dist < info.center_dispacement ) )
	
	#returns true if the distance is less than a threshold
	return ( dist < info.center_dispacement )

##
#	\brief print the task's information
#	\param task_name : String
#   \param task : command 
#	\return : None
# 	
#	This function just prints information about the tasks, such as which arm
#   which position and which cube.
def print_task( task_name, task ):
	if task is not None:
		rospy.loginfo( " [task manager] TASK [%s] arm(%s) pos(%s) cube(%s)", task_name, task.arm, task.pos, task.cube )
	else:
		rospy.loginfo( " [task manager] TASK [%s] is None.", task_name )

## --------------------------- TASK MANAGER

##
#	\brief operating loop of the task manager, it continues until there are still blue blocks
#	\param : None
#	\return : None
# 	
#	This function searches for a goal for both the left and the right arm,
#   if we found at least one task we execute it, if not we wait for the 
#   situation to change. We repeat this until we have blue blocks available.
#   When there are no more blue blocks I close the node.
def task_manager_body():
	global task_arm_left, task_arm_right, frame_block_red, miss
	
	task_manager_frequency = rospy.Rate(info.task_manager_freq)
	
	while( len(frame_block_blue) > 0 and not rospy.is_shutdown() ):
		# get a goal for the left arm if possible
		task_arm_left = get_task_arm_left()
		print_task( "task_arm_left", task_arm_left )
		
		# get a goal for the right arm if possible
		task_arm_right = get_task_arm_right()
		print_task( "task_arm_right", task_arm_right )
		
		# execute commands 
		if( task_arm_left != None or task_arm_right != None ):
			task_execute_wrapper(task_arm_right, task_arm_left)
		else:
			# wait for the situation to change
			rospy.loginfo( " [task manager] Nothing to do. Waiting..." )
			task_manager_frequency.sleep()
	
	if rospy.is_shutdown():
		rospy.loginfo( " [task manager] shutdown signal received." )
	else:
		rospy.loginfo( " [task manager] Job done. (misses: %d)", miss )

##
#	\brief get a goal for the right arm
#	\param : None
#	\return : None if there are no tasks, the task otherwise
# 	
#	This function searches for a goal for the right arm checking first if there 
#   is a block already in the center. If that is the case the right arm does
#   nothing. If the center is free I exclude from the choice all the blocks
#   too close to the operator and the blocks not accessible. From the remaining
#   blocks I pick the one with the greatest distance from the hand of the operator.
#   At the end it sets the message to send to the server.
def get_task_arm_right():
	global frame_block_blue, frame_block_red, frame_goal_red, block_at_center
	
	# check if there are already blocks in the center
	if block_at_center is not None:
		return None
	
	# get the position of the nearest hand
	frame_human_hand = get_nearest_hand_pose( frame_goal_red )
	
	# do nothing if the list of the blue blocks is empty
	if( len(frame_block_blue) == 0 ):
		return None
		
	hand_on_right = block_on_right_table(( "hand", frame_human_hand ))
	
	# position of the blue blocks
	bluebox = frame_block_blue.items()
	
	# get all the blocks on the right side of the table
	bluebox = list(filter( block_on_right_table, bluebox ))
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) blue blocks to evaluate: %s", "get_task_arm_right", str(dict(bluebox).keys()) )
	 
	# if the list contains only one block go directly after the search of a block 
	
	if ( len(bluebox) > 1 ):
		# exclude all the blocks too close to the operator hand
		if( len(bluebox) > 0 ):
			bluebox = list(filter( block_not_near_to_hand, bluebox ))
		
		# exclude all non accessible blocks
		if( len(bluebox) > 0 and len(frame_block_red) > 0 ):
			# redbox  = frame_block_red.items()
			bluebox = list(filter( block_is_accessible, bluebox ))
		
		# If the lenght is greater than 1 sort the blocks with respect 
		#to the distance from the human hand
		if ( len(bluebox) > 1 and hand_on_right ):
			bluebox.sort(key=dist_block_hand, reverse=True)
	
	# find the best block
	if ( len(bluebox) > 0 ):
		# rospy.loginfo( " [task manager] (%s) found: %s", "get_task_arm_right", str(dict(bluebox).keys()) )
		# get the first element of the list and return the task
		winner = list(bluebox)[0][0]
		right_task = commandRequest()
		right_task.arm = "right"
		right_task.pos = "center"
		right_task.cube = winner
		return right_task
	else:
		# there aren't any blocks available
		return None

##
#	\brief get a goal for the left arm
#	\param : None
#	\return : None if there are no tasks, the task otherwise
# 	
#	This function searches for a goal for the left arm excluding all blocks
#   that are too close to the hand of the operator and all non accessible blocks. 
#   From the remaining ones I prioritize the block in the center and then
#   only if no blocks are in the center it picks the furthest one from the 
#   hand of the operator. At the end it sets the message to send to the server.
def get_task_arm_left():
	global frame_block_blue, frame_block_red, frame_goal_blue, block_at_center
	
	# do nothing if the list of the blue blocks is empty 
	if( len(frame_block_blue) == 0 ):
		return None
	
	# get the position of the nearest hand
	frame_human_hand = get_nearest_hand_pose( frame_goal_blue )
	hand_on_left = block_on_left_table(( "hand", frame_human_hand ))
	
	# position of the blue blocks
	bluebox = frame_block_blue.items()
	
	# get all the blocks on the left side of the table 
	bluebox = list(filter( block_on_left_table, bluebox ))
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) blue blocks to evaluate: %s", "get_task_arm_left", str(dict(bluebox).keys()) )
	
	# if the list contains only one block go directly after the search of a block
	
	winner = None
	
	if ( len(bluebox) > 1 ):
		# exclude all the blocks too close to the operator hand
		if( len(bluebox) > 0 ):
			bluebox = list(filter( block_not_near_to_hand, bluebox ))
		
		# exclude all non accessible blocks
		if( len(bluebox) > 0 and len(frame_block_red) > 0 ):
			# redbox  = frame_block_red.items()
			bluebox = list(filter( block_is_accessible, bluebox ))
		
		# If the lenght is greater than 1 sort the blocks with respect 
		#to the distance from the human hand
		if ( len(bluebox) >= 1 and ( block_at_center is not None ) and ( ( block_at_center, frame_block_blue[block_at_center] ) in list(bluebox) ) ):
			# pick the central block if one is there
			winner = block_at_center
			rospy.loginfo( " [task manager] (%s) forcing block at centre: %s", "get_task_arm_left", block_at_center )
		elif ( len(bluebox) > 1 and hand_on_left ):
			bluebox.sort(key=dist_block_hand, reverse=True)
			# pick the first block of the list
			winner = list(bluebox)[0][0]
			rospy.loginfo( " [task manager] (%s) winner: %s", "get_task_arm_left", winner )
	
	# get the chosen block
	if( winner is None and len(bluebox) == 1 ):
		winner = list(bluebox)[0][0]
	if ( winner is not None ):
		# send the command
		# rospy.loginfo( " [task manager] (%s) found: %s", "get_task_arm_left", str(dict(bluebox).keys()) )
		left_task = commandRequest()
		left_task.arm = "left"
		left_task.pos = "box"
		left_task.cube = winner
		return left_task
	else:
		# there aren't any blocks available
		return None

##
#	\brief decide whether to use the sequential or parallel execution
#	\param task_right the task planned for the right arm
#   \param task_left the task planned for the left arm
#	\return : None 
# 	
#	This function checks if we have a task for both arms, and saves the position
#   of the blocks that needs to be picked up from the tasks in two varibales.
#   If we have to execute a task with both arms I check if the distance 
#   between the two blocks is greater than a given threshold, in that case it
#   calls the execution of the two arms in parallel while if they are too
#   close it launches first the one of the right arm and after the one of the left arm.
def task_execute_wrapper( task_right, task_left ):
	global frame_block_blue
	
	if ( task_right is not None ):
		label_A = frame_block_blue[ task_right.cube ]
	if ( task_left is not None ):
		label_B = frame_block_blue[ task_left.cube ]
	
	if ( ( task_right is None ) and ( task_left is None ) ):
		# if there is nothing to do both are None
		pass
	elif ( ( task_right is not None ) and ( task_left is not None ) ):
		if ( distance_between( label_A, label_B ) > info.minimum_distance_parallel ):
			rospy.loginfo( " [task manager] execution: parallel" )
			task_execute( task_right, task_left )
		else:
			rospy.loginfo( " [task manager] execution: sequential" )
			task_execute( task_right, None )
			task_execute( None, task_left )
	else:
		rospy.loginfo( " [task manager] execution: only %s", ( "right" if ( task_right is not None ) else "left" ) )
		task_execute( task_right, task_left )

##
#	\brief execute the task
#	\param task_right the task planned for the right arm
#   \param task_left the task planned for the left arm
#	\return : it will return true if it was successfull, false if that is not the case.
# 	
#	This function sends the tasks to the planner and returns false if the call 
#   has not been executed, it sends the command for the left and the right.
#   In order to see if the robot has completed the task given I check if 
#   the arm has reached the home position, this control is implemented by
#   calling the server baxter_at_home.
#   I also implement a check on the final position of the block to see
#   if it was successful.
def task_execute( task_right, task_left ):
	global srv_controller, srv_baxter_home, frame_block_blue, miss, block_at_center
	
	rospy.loginfo( " [task manager] sending tasks to the controller..." )
	
	right_at_home = False
	left_at_home = False
	left_box = ""
	
	# send the left task to the controller
	if task_left is not None:
		left_box = task_left.cube
		res = srv_controller( task_left )
		if not res.ok:
			rospy.loginfo( " [task manager] CONTROLLER FAILURE in sending left task" )
			miss = miss + 1
			if task_right is None:
				return False
			else:
				left_at_home = True
	else:
		left_at_home = True
	
	# send the right task to the controller
	if task_right is not None:
		res = srv_controller( task_right )
		if not res.ok:
			rospy.loginfo( " [task manager] CONTROLLER FAILURE in sending right task" )
			miss = miss + 1
			if not left_at_home:
				return False
			else:
				right_at_home = True
				rospy.loginfo( " [task manager] executing only left task. " )
	else:
		right_at_home = True
	
	# if both task fail close the controller
	if right_at_home and left_at_home:
		return False
	
	# wait for baxter to start moving
	baxter_is_moving = False
	rospy.loginfo( " [task manager] waiting for BAXTER to start movement..." )
	while not baxter_is_moving:
		baxter_is_moving = not ( ( srv_baxter_home( arm="right" ) ).at_home and ( srv_baxter_home( arm="left" ) ).at_home )
	rospy.loginfo( " [task manager] BAXTER is moving; waiting for the tasks..." )
	
	# wait for the robto to reach the start position
	f = rospy.Rate( 10 )
	while not ( right_at_home and left_at_home ):
		# wait
		f.sleep()
		
		# check the position of the two arms
		if not right_at_home:
			req = at_homeRequest()
			req.arm = "right"
			right_at_home = srv_baxter_home( req )
			right_at_home = right_at_home.at_home
		if not left_at_home:
			req = at_homeRequest()
			req.arm = "left"
			left_at_home = srv_baxter_home( req )
			left_at_home = left_at_home.at_home
	
	baxter_is_moving = False
	
	# check the new position of the blue block
	if task_left is not None:
		rospy.loginfo( " [task manager] checking if %s has reached the goal... ", left_box )
		if block_in_blue_goal( left_box ):
			# if the blue box is in the goal delete it from the list
			rospy.loginfo( " [task manager] -> BLUE block %s in goal", left_box )
			
			# check if the block was in the center
			if (block_at_center is not None) and ( block_at_center == left_box ):
				block_at_center = None
			
			# delete the block from the list
			del frame_block_blue[ left_box ]
			
		else:
			miss = miss + 1
			rospy.loginfo( " [task manager] -> BLUE block %s: failed. Retry. (misses: %d)", left_box, miss )
	
	# check if the block is in the center
	if task_right is not None:
		if check_block_is_at_center( frame_block_blue[task_right.cube] ):
			#the block is in the center
			block_at_center = task_right.cube
		else:
			miss = miss + 1
			rospy.loginfo( " [task manager] -> BLUE block %s: failed. Retry. (misses: %d)", task_right.cube, miss )
	
	rospy.loginfo( " [task manager] task execution ... done" )
	
	return True

## --------------------------- NODE

##
#	\brief prints information about the state of the node
#	\param None
#	\return : None
#
def on_shutdown_msg():
    rospy.loginfo( " [task manager] offline" )

##
#	\brief main function
#	\param None
#	\return : None
#
#    Initializes the ros node and also the subscribers and services needed.
#    It updates the position of all the blocks and it starts the funcitoning loop.
def main():
    global srv_controller, msg_unity_tf, srv_baxter_home, update_counter
    global frame_block_blue, frame_block_red
    
    print( "progetto SKYNET avviato." )
    
    # node init
    rospy.init_node( "task_manager" )
    rospy.on_shutdown( on_shutdown_msg )
    
    # subscribe: unity_tf o tf depending on settings
    if info.use_unity:
        rospy.loginfo( " [task manager] subscribe to unity_tf: %s ...", info.topic_unity_tf )
        unity_tf = rospy.Subscriber( info.topic_unity_tf, UnityTf, callback_unity )
        rospy.loginfo( " [task manager] subscribe to unity_tf: %s ... OK!", info.topic_unity_tf )
    else:
        rospy.loginfo( " [task manager] subscribe to tf: %s ...", info.topic_tf )
        unity_tf = rospy.Subscriber( info.topic_tf, TFMessage, callback_tf )
        rospy.loginfo( " [task manager] subscribe to tf: %s ... OK!", info.topic_tf )
    
    # service: controller_server
    rospy.loginfo( " [task manager] service: %s ...", info.server_movement_controller )
    rospy.wait_for_service( info.server_movement_controller )
    srv_controller = rospy.ServiceProxy( info.server_movement_controller, command )
    rospy.loginfo( " [task manager] service: %s ... OK!", info.server_movement_controller )
    
    # service: baxter_at_home_server
    rospy.loginfo( " [task manager] service: %s ...", info.server_baxter_at_home )
    rospy.wait_for_service( info.server_baxter_at_home )
    srv_baxter_home = rospy.ServiceProxy( info.server_baxter_at_home, at_home )
    rospy.loginfo( " [task manager] service: %s ... OK!", info.server_baxter_at_home )
    
    # wait for first update
    rospy.loginfo( " [task manager] waiting for the first update... (bound: %d)", ( 1 if info.use_unity else 125 ) )
    while update_counter < ( 1 if info.use_unity else 125 ):
        (rospy.Rate(1)).sleep()
    rospy.loginfo( " [task manager] waiting for the first update... OK! " )
    
    labels = list(frame_block_blue.keys())
    for key in labels:
        if block_in_blue_goal( key ):
            rospy.loginfo( " [task manager] -> BLUE block %s in goal", key )
            del frame_block_blue[ key ]
    
    # functioning loop
    rospy.loginfo( " [task manager] online" )
    task_manager_body()

if __name__ == "__main__":
    main()

