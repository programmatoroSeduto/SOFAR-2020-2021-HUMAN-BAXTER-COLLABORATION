#! /usr/bin/env python

import rospy
from controller_baxter.srv import at_home, at_homeRequest, at_homeResponse
from controller_baxter.srv import command, commandRequest, commandResponse
from human_baxter_collaboration.msg import UnityTf
from geometry_msgs.msg import Point, PoseStamped, Pose

import math

import controller_baxter.sim_infos as info



## --------------------------- GLOBALI 

# posizione della mano dell'operatore
frame_human_hand = Pose()


# posizione di tutti i blocchi rossi
frame_block_red = { "A" : Pose(), "L" : Pose(), "B" : Pose(), "H" : Pose(), "D" : Pose(), "F" : Pose() }


# posizione di tutti i blocchi blu
frame_block_blue = { "E" : Pose(), "C" : Pose(), "I" : Pose(), "G" : Pose(), "M" : Pose() }


# frame del goal blu
frame_goal_blue = Pose()
frame_goal_blue.position.x = -0.608
frame_goal_blue.position.y = 0.801
frame_goal_blue.position.z = 0.668


# frame del goal rosso
frame_goal_red = Pose()
frame_goal_red.position.x = 0.597
frame_goal_red.position.y = 0.801
frame_goal_red.position.z = 0.743


# posizione del tavolo
frame_table = Pose()
frame_table.position.x = 0
frame_table.position.y = 0.4
frame_table.position.z = 0.7673182



## --------------------------- TOPIC E SERVER

# servizio "controller_server"
srv_controller = None


# servizio "baxter_at_home_server"
srv_baxter_home = None


# topic "unity_tf"
msg_unity_tf = None


# callback topic unity
def callback_unity( data ):
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# etichette dei blocchi
	labels_red = frame_block_red.keys()
	labels_blue = frame_block_blue.keys()
	
	for i in range( len(data.frames) ):
		# etichetta del frame da Unity
		name = data.frames[i].header.frame_id
		
		if ( name in frame_block_red ):
			frame_block_red[ name ] = data.frames[i].pose
		elif ( name in frame_block_blue ):
			frame_block_blue[ name ] = data.frames[i].pose
		elif ( name == info.name_human_hand ):
			frame_human_hand = data.frames[i].pose
		else:
			# ignora il messaggio
			pass



## --------------------------- FUNZIONI

# il blocco rosso si trova nel goal?
# elem : String
def block_in_red_goal( elem ):
	global frame_block_red, frame_goal_red
	
	# il blocco esiste
	# SOLO SE questa funzione viene richiamata da
	#    task_manager_body
	# altrimenti potrebbe sollevare eccezione
	
	# posizione del blocco e del goal
	P = [ frame_block_red[elem].position.x, frame_block_red[elem].position.z ]
	O = [ frame_goal_red.position.x, frame_goal_red.position.z ]
	
	# ritora true se il blocco si trova all'interno della scatola
	if( P[0] < (O[0] - info.sz_goal[0]/2 ) or P[0] > (O[0] + info.sz_goal[0]/2 ) ):
		return False
	if( P[1] < (O[1] - info.sz_goal[2]/2 ) or P[1] > (O[1] + info.sz_goal[2]/2 ) ):
		return False
	
	return True


# il blocco blu si trova nel goal?
# elem : String
def block_in_blue_goal( elem ):
	global frame_block_blue, frame_goal_blue
	
	# supponiamo che il blocco esista nella lista
	# SOLO SE questa funzione viene utilizzata da task_execute()
	# e non richiamata da altre parti del codice
	
	# posizione del blocco e del goal
	P = [ frame_block_blue[elem].position.x, frame_block_blue[elem].position.z ]
	O = [ frame_goal_blue.position.x, frame_goal_blue.position.z ]
	
	# ritora true se il blocco si trova all'interno della scatola
	if( P[0] < (O[0] - info.sz_goal[0]/2 ) or P[0] > (O[0] + info.sz_goal[0]/2 ) ):
		return False
	if( P[1] < (O[1] - info.sz_goal[2]/2 ) or P[1] > (O[1] + info.sz_goal[2]/2 ) ):
		return False
	
	return True


# Il blocco si trova sul lato destro del tavolo?
#  elem : ( 'name', Pose )
def block_on_right_table( elem ):
	global frame_table
	
	P = [ elem[1].position.x, elem[1].position.z ]
	O = [ frame_table.position.x, frame_table.position.z ]
	
	if( P[0] > O[0] or P[0] < (O[0] - info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True


# segnala se il blocco e' troppo vicino all'operatore
#  elem : ( 'name', Pose )
def block_not_near_to_hand( elem ):
	global frame_human_hand
	
	#  calcola la distanza tra l'origine della mano e quella del blocco
	P = [ elem[1].position.x, elem[1].position.z ]
	O = [ frame_human_hand.position.x, frame_human_hand.position.z ]
	dist = math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )
	
	return True if ( dist >= info.sz_hand ) else False


# il blocco e' accessibile?
#  elem : ( 'name', Pose )
def block_is_accessible( elem_b ):
	global frame_block_red

	for elem_r in frame_block_red.values():
		dx = abs( elem_b[1].position.x - elem_r.position.x )
		dz = abs( elem_b[1].position.z - elem_r.position.z )
		# se dy < 0 allora il blocco rosso si trova sopra quello blu
		dy = elem_b[1].position.y - elem_r.position.y 
		
		if (dx < info.sz_cube and dz < info.sz_cube and dy < 0):
			return False

	return True


# distanza tra un blocco e la mano dell'operatore
#  elem : ( 'name', Pose )
def dist_block_hand( elem ):
	global frame_human_hand
	
	#  calcola la distanza tra l'origine della mano e quella del blocco
	P = [ elem[1].position.x, elem[1].position.z ]
	O = [ frame_human_hand.position.x, frame_human_hand.position.z ]
	
	return math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )


# Il blocco si trova sul lato destro del tavolo?
#  elem : ( 'name', Pose )
def block_on_left_table( elem ):
	global frame_table
	
	P = [ elem[1].position.x, elem[1].position.z ]
	O = [ frame_table.position.x, frame_table.position.z ]
	
	if( P[0] < O[0] or P[0] > (O[0] + info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True


# stampa a video i dati del task
# task_name : String
# task : command 
def print_task( task_name, task ):
	if task is not None:
		rospy.loginfo( " [task manager] TASK [name:%s] arm(%s) pos(%s) cube(%s)", task_name, task.arm, task.pos, task.cube )
	else:
		rospy.loginfo( " [task manager] TASK [name:%s] is None.", task_name )



## --------------------------- TASK MANAGER

# ciclo di funzionamento del task manager
# continua fin quando ci sono blocchi blu da spostare, poi chiudi
def task_manager_body():
	global task_arm_left, task_arm_right, frame_block_red
	
	task_manager_frequency = rospy.Rate(info.task_manager_freq)
	
	while( len(frame_block_blue) > 0 ):
		# ottieni un compito per il braccio sinistro se possibile
		task_arm_left = get_task_arm_left()
		print_task( "task_arm_left", task_arm_left )
		
		# ottieni un compito per il braccio destro se possibile
		task_arm_right = get_task_arm_right()
		print_task( "task_arm_right", task_arm_right )
		
		# esegui i comandi 
		if( task_arm_left != None or task_arm_right != None ):
			task_execute(task_arm_right, task_arm_left)
			
			# elimina i blocchi rossi non più da considerare, se possibile
			frame_block_red = list(filter( frame_block_red, block_in_red_goal ))
		else:
			# attendi che la situazione cambi
			rospy.loginfo( " [task manager] Nothing to do. Waiting..." )
			task_manager_frequency.sleep()
	
	rospy.loginfo( " [task manager] Job done." )


# ottieni un compito per il braccio destro
def get_task_arm_right():
	global frame_block_blue, frame_block_red, frame_human_hand
	
	# non avviare la procedura se la lista è vuota. 
	if( len(frame_block_blue) == 0 ):
		return None
		
	hand_on_right = block_on_right_table(( "__", frame_human_hand ))
	
	# posizione dei box blu
	bluebox = frame_block_blue.items()
	
	# se la lista contiene un solo elemento, salta direttamente a dopo la 
	# procedura di ricerca del blocco. 
	
	if ( len(bluebox) > 1 ):
		# escludi tutti i blocchi sul lato destro del tavolo 
		bluebox = list(filter( block_on_right_table, bluebox ))
		
		# escludi tutti i blocchi troppo vicini alla mano dell'operatore
		if( len(bluebox) > 0 ):
			bluebox = list(filter( block_not_near_to_hand, bluebox ))
		
		# escludi i blocchi non accessibili
		if( len(bluebox) > 0 and len(frame_block_red) > 0 ):
			# redbox  = frame_block_red.items()
			bluebox = list(filter( block_is_accessible, bluebox ))
		
		# ordina i blocchi in base a quello più lontano dalla mano
		# ordina solo se la len è maggiore di 1
		if ( len(bluebox) > 1 and hand_on_right ):
			bluebox.sort(key=dist_block_hand, reverse=True)
	
	# chi sarà il blocco adatto?
	if ( len(bluebox) > 0 ):
		# prendi il primo blocco della lista e ritorna il task
		winner = bluebox[0][0]
		right_task = commandRequest()
		right_task.arm = "right"
		right_task.pos = "center"
		right_task.cube = winner
		return right_task
	else:
		# nessun blocco compatibile
		return None


# ottieni un compito per il braccio sinistro
def get_task_arm_left():
	global frame_block_blue, frame_block_red, frame_human_hand
	
	# non avviare la procedura se la lista è vuota. 
	if( len(frame_block_blue) == 0 ):
		return None
	
	hand_on_left = block_on_left_table(( "   ;-)   ", frame_human_hand ))
	
	# posizione dei box blu
	bluebox = frame_block_blue.items()
	
	# se la lista contiene un solo elemento, salta direttamente a dopo la 
	# procedura di ricerca del blocco. 
	
	if ( len(bluebox) > 1 ):
		# escludi tutti i blocchi sul lato sinistro del tavolo 
		bluebox = list(filter( block_on_left_table, bluebox ))
		
		# escludi tutti i blocchi troppo vicini alla mano dell'operatore
		if( len(bluebox) > 0 ):
			bluebox = list(filter( block_not_near_to_hand, bluebox ))
		
		# escludi i blocchi non accessibili
		if( len(bluebox) > 0 and len(frame_block_red) > 0 ):
			# redbox  = frame_block_red.items()
			bluebox = list(filter( block_is_accessible, bluebox ))
		
		# ordina i blocchi in base a quello più lontano dalla mano
		# ordina solo se la len è maggiore di 1
		if ( len(bluebox) > 1 and hand_on_left ):
			bluebox.sort(key=dist_block_hand, reverse=True)
	
	# chi sarà il blocco adatto?
	if ( len(bluebox) > 0 ):
		# prendi il primo blocco della lista e ritorna il task
		winner = bluebox[0][0]
		left_task = commandRequest()
		left_task.arm = "left"
		left_task.pos = "box"
		left_task.cube = winner
		return left_task
	else:
		# nessun blocco compatibile
		return None


# esegui i task
def task_execute( task_right, task_left ):
	global srv_controller, srv_baxter_home, frame_block_blue
	
	rospy.loginfo( " [task manager] executing tasks..." )
	
	right_at_home = False
	left_at_home = False
	left_box = ""
	
	
	# Invio delle richieste
	if task_left is not None:
		left_box = task_left.cube
		res = srv_controller( task_left )
	else:
		left_at_home = True
	
	if task_right is not None:
		res = srv_controller( task_right )
	else:
		right_at_home = True
	
	
	# attendi che il robot abbia raggiunto la posizione iniziale
	f = rospy.Rate( 10 )
	while not ( right_at_home and left_at_home ):
		# attesa
		f.sleep()
		
		# verifica la posizione dei due bracci
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
	
	
	# verifica la nuova posizione del blocco blu
	if task_left is not None:
		if block_in_blue_goal( left_box ):
			# elimina il blocco dalla raccolta
			rospy.loginfo( " [task manager] block %s in goal. ", left_box )
			del frame_block_blue[ left_box ]



## --------------------------- NODO

def on_shutdown_msg():
    rospy.loginfo( " [task manager] task_manager offline" )


def main():
    global srv_controller, msg_unity_tf, srv_baxter_home
    
    # node init
    rospy.init_node( "task_manager" )
    rospy.on_shutdown( on_shutdown_msg )
    
    # subscribe: unity_tf
    rospy.loginfo( " [task manager] subscribe: %s ...", info.topic_unity_tf )
    unity_tf = rospy.Subscriber( info.topic_unity_tf, UnityTf, callback_unity )
    rospy.loginfo( " [task manager] subscribe: %s ... OK!", info.topic_unity_tf )
    
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
    
    # ciclo di funzionamento
    rospy.loginfo( " [task manager] task_manager online" )
    task_manager_body()



if __name__ == "__main__":
    main()

