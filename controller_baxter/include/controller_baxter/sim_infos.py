#!/usr/bin/env python

## --------------------------- ALTRI PARAMETRI

# frequenza del task manager
task_manager_freq = 2

# frequenza del log da Unity
#    ogni quanti messaggi ritornare un'informazione sul topic unity_tf
log_freq = 250

# frequenza del log completo da Unity
#    ogni quanti messaggi ritornare un'informazione sul topic unity_tf
log_freq_extended = 1000000

# distanza minima che i due blocchi devono avere per l'esecuzione parallela
minimum_distance_parallel = 0.5

# scostamento della posizione del blocco dal centro
center_dispacement = 0.5



## --------------------------- DIMENSIONI OGGETTI

# lato del cubetto (tutti i lati uguali)
sz_cube = 0.05

# dimensioni x e y del goal (x, y, z)
sz_goal = [ 0.14, 0.01, 0.221 ]

# dimensioni del tavolo (x, y, z)
sz_table = [ 2, 0.8, 1 ]

# dimensioni della mano (soglia di allarme)
sz_hand = 0.3



## --------------------------- NOMI

# mani
name_human_hand_right = "hand_r"
name_human_hand_left = "hand_l"

# goal
name_goal_blue = "Bluebox"
name_goal_red = "Redbox"

# server movement controller
server_movement_controller = "controller_server"

# server di baxter_at_home
server_baxter_at_home = "baxter_at_home_server"

# canale da Unity
topic_unity_tf = "unity_tf"
