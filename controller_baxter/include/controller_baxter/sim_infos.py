#!/usr/bin/env python

## --------------------------- OTHER PARAMETERS

# frequency of the task manager
task_manager_freq = 2

# frequency of the log from unity
#    how many messages before returning un information on the topic unity_tf
#    NOTE: using the normal tf, it is needed the use of 'use_verbose' otherwise nothing will be printed
log_freq = 250

# frequency of the complete log from unity
#    how many messages before returning un information on the topic unity_tf
#    NOTE: using the normal tf, it is needed the use of 'use_verbose' otherwise nothing will be printed
log_freq_extended = 1000000

# minimum distance two blocks must have for the parallel execution
minimum_distance_parallel = 0.5

# deviation of the position of the block in the center
center_dispacement = 0.5

# if we want to use the topic from unity or directly from tf
use_unity = False

# "verbose" modality: print detailed log on consolle
use_verbose = False



## --------------------------- OBJECT DIMENSION

# size of the side of the block (all sides are equal)
sz_cube = 0.05

# dimension x,y,z of the goal
sz_goal = [ 0.14, 0.01, 0.221 ]

# dimension of the table (x,y,z)
sz_table = [ 2, 0.8, 1 ]

# dimension of the hand, allarm size
sz_hand = 0.3



## --------------------------- Names

# hands
name_human_hand_right = "hand_r"
name_human_hand_left = "hand_l"

# goal
name_goal_blue = "Bluebox"
name_goal_red = "Redbox"

# server movement controller
server_movement_controller = "controller_server"

# server baxter_at_home
server_baxter_at_home = "baxter_at_home_server"

# channel from Unity
topic_unity_tf = "unity_tf"
topic_tf = "tf"
