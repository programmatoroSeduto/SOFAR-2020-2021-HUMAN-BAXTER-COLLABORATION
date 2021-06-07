#!/usr/bin/env python

## --------------------------- ALTRI PARAMETRI

# frequenza del task manager
task_manager_freq = 2



## --------------------------- DIMENSIONI OGGETTI

# lato del cubetto (tutti i lati uguali)
sz_cube = 0.05

# dimensioni x e y del goal (x, y, z)
sz_goal = [ 0.015, 1, 0.023 ]

# dimensioni del tavolo (x, y, z)
sz_table = [ 2, 0.8, 0.6 ]

# dimensioni della mano (soglia di allarme)
sz_hand = 0.19



## --------------------------- NOMI

# nomi
name_human_hand = "hand_r"
name_goal_blue = "Bluebox"
name_goal_red = "Redbox"

# server movement controller
server_movement_controller = "controller_server"

# server di baxter_at_home
server_baxter_at_home = "baxter_at_home_server"

# canale da Unity
topic_unity_tf = "unity_tf"
