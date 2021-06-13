#! /usr/bin/env python

import rospy
from controller_baxter.srv import at_home, at_homeRequest, at_homeResponse
from controller_baxter.srv import command, commandRequest, commandResponse
from human_baxter_collaboration.msg import UnityTf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Transform, Vector3, Quaternion

import math
import sys

import controller_baxter.sim_infos as info



## --------------------------- GLOBALI 

# posizione dellr mani dell'operatore
frame_human_hand_right = Pose()
frame_human_hand_left = Pose()


# posizione di tutti i blocchi rossi
frame_block_red = { "A" : Pose(), "L" : Pose(), "B" : Pose(), "H" : Pose(), "D" : Pose(), "F" : Pose() }


# posizione di tutti i blocchi blu
frame_block_blue = { "E" : Pose(), "C" : Pose(), "I" : Pose(), "G" : Pose(), "M" : Pose() }


# frame del goal blu
frame_goal_blue = Pose()


# frame del goal rosso
frame_goal_red = Pose()


# posizione del tavolo
frame_table = Pose()
frame_table.position.x = 0.5
frame_table.position.y = 0
frame_table.position.z = 0


# quante volte il movimento è fallito
miss = 0


# centro occupato?
#    string / None
block_at_center = None



## --------------------------- TOPIC E SERVER

# contatore degli aggiornamenti
update_counter = 0


# servizio "controller_server"
srv_controller = None


# servizio "baxter_at_home_server"
srv_baxter_home = None


# topic "unity_tf"
msg_unity_tf = None


# log di fine aggiornamento
def update_log( ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# check e contatore aggiornamenti
	update_counter = update_counter + 1
	
	if( update_counter % info.log_freq == 0 ):
		rospy.loginfo( " [task manager] update no. %d", update_counter )
		rospy.loginfo( " [task manager]     %s len:%d", "frame_block_blue", len( frame_block_blue ) )
		rospy.loginfo( " [task manager]     %s len:%d", "frame_block_red", len( frame_block_red ) )
	if( update_counter % info.log_freq_extended == 0 ):
		rospy.loginfo( " [task manager] update no. %d EXTENDED LOG", update_counter )
		rospy.loginfo( " [task manager] frame_block_red:\n========\n %s\n========\n", str( frame_block_red ) )
		rospy.loginfo( " [task manager] frame_block_blue:\n========\n %s\n========\n", str( frame_block_blue ) )


# aggiorna la posizione dei blocchi rossi
def update_red_blocks( ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# elimina i blocchi rossi già nel goal
	labels = list(frame_block_red.keys())
	for elem in labels:
		if block_in_red_goal( elem ):
			rospy.loginfo( " [task manager] -> RED block %s in goal", elem )
			del frame_block_red[ elem ]
			if len( frame_block_red ) == 0 :
				rospy.loginfo( " [task manager] human job done." )


# cast oggetto TransformStamped in Pose
def tranformStamped_to_pose( data ):
	to_return = Pose()
	to_return.orientation = data.transforms[0].transform.rotation
	to_return.position.x = data.transforms[0].transform.translation.x
	to_return.position.y = data.transforms[0].transform.translation.y
	to_return.position.z = data.transforms[0].transform.translation.z
	
	return to_return


# callback topic unity
def callback_unity( data ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# etichette dei blocchi
	labels_red = frame_block_red.keys()
	labels_blue = frame_block_blue.keys()
	
	for i in range( len(data.frames) ):
		# etichetta del frame da Unity
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
			# ignora il messaggio
			pass
	
	# elimina i blocchi rossi già nel goal
	update_red_blocks( )
	
	# log di fine aggiornamento
	update_log( )


# callback tf
def callback_tf( data ):
	global update_counter
	global frame_human_hand, frame_block_red, frame_block_blue, frame_table, frame_goal_blue, frame_goal_red
	
	# etichette dei blocchi
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
		# ignora il messaggio
		pass
	
	# log di fine aggiornamento
	if info.use_verbose:
		update_log( )
	else:
		update_counter = update_counter + 1



## --------------------------- FUNZIONI

# distanza tra due pose qualunque
def distance_between( pose_A, pose_B ):
	A = [ pose_A.position.y, pose_A.position.x ]
	B = [ pose_B.position.y, pose_B.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) A[%f, %f] B[%f, %f]", "distance_between", A[0], A[1], B[0], B[1] )
	
	return math.sqrt( (B[0]-A[0])**2 + (B[1]-A[1])**2 )


# il blocco rosso si trova nel goal?
# elem : String
def block_in_red_goal( elem ):
	global frame_block_red, frame_goal_red
	
	# il blocco esiste
	# SOLO SE questa funzione viene richiamata da
	#    task_manager_body
	# altrimenti potrebbe sollevare eccezione
	
	# posizione del blocco e del goal
	P = [ frame_block_red[elem].position.y, frame_block_red[elem].position.x ]
	O = [ frame_goal_red.position.y, frame_goal_red.position.x ]
	
	if info.use_verbose:
		#rospy.loginfo( " [task manager] (%s) %s[%f, %f] 0[%f, %f] x in [%f, %f] y in [%f, %f]", "block_in_red_goal", elem, P[0], P[1], O[0], O[1], O[0] - info.sz_goal[0]/2, O[0] + info.sz_goal[0]/2, O[1] - info.sz_goal[2]/2, O[1] + info.sz_goal[2]/2 )
		pass
	
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
	P = [ frame_block_blue[elem].position.y, frame_block_blue[elem].position.x ]
	O = [ frame_goal_blue.position.y, frame_goal_blue.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] 0[%f, %f] x in [%f, %f] y in [%f, %f]", "block_in_blue_goal", elem, P[0], P[1], O[0], O[1], O[0] - info.sz_goal[0]/2, O[0] + info.sz_goal[0]/2, O[1] - info.sz_goal[2]/2, O[1] + info.sz_goal[2]/2 )
	
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
	
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_table.position.y, frame_table.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | x NOT in [%f, %f] ? | y NOT in [%f, %f] ? | test: (%s, %s)", "block_on_right_table", elem[0], P[0], P[1], O[0], O[1], O[0], O[0] + info.sz_table[0]/2, O[1] - info.sz_table[2]/2, O[1] + info.sz_table[2]/2, str( P[0] > O[0] or P[0] < (O[0] - info.sz_table[0]/2) ), str( P[1] < (O[1] - info.sz_table[2]/2) or P[1] > (O[1] + info.sz_table[2]/2) ) )
	
	if( P[0] > O[0] or P[0] < (O[0] - info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True


# Il blocco si trova sul lato destro del tavolo?
#  elem : ( 'name', Pose )
def block_on_left_table( elem ):
	global frame_table
	
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_table.position.y, frame_table.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | x NOT in [%f, %f] ? | y NOT in [%f, %f] ? | test: (%s %s)", "block_on_left_table", elem[0], P[0], P[1], O[0], O[1], (O[0] - info.sz_table[0]/2), O[0], O[1] - info.sz_table[2]/2, O[1] + info.sz_table[2]/2, str( P[0] < O[0] or P[0] > (O[0] + info.sz_table[0]/2) ), str( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ) )
	
	if( P[0] < O[0] or P[0] > (O[0] + info.sz_table[0]/2) ):
		return False
	if( P[1] < (O[1] - info.sz_table[2]/2) or P[1] >  (O[1] + info.sz_table[2]/2) ):
		return False
	
	return True


# ottieni la mano più vicina alla pose considerata
# elem: Pose
def get_nearest_hand_pose( elem ):
	global frame_human_hand_right, frame_human_hand_left
	frame_human_hand = None
	if( distance_between( elem, frame_human_hand_left ) > distance_between( elem, frame_human_hand_right ) ):
		return frame_human_hand_right
	else:
		return frame_human_hand_left


# segnala se il blocco e' troppo vicino all'operatore
#  elem : ( 'name', Pose )
def block_not_near_to_hand( elem ):
	# cerca la mano più vicina al blocco considerato
	frame_human_hand = get_nearest_hand_pose( elem[1] )
	
	#  calcola la distanza tra l'origine della mano e quella del blocco
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_human_hand.position.y, frame_human_hand.position.x ]
	dist = math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | dist: %f | limit: %f | result: %s", "block_not_near_to_hand", elem[0], P[0], P[1], O[0], O[1], dist, info.sz_hand, str( dist >= info.sz_hand ) )
	
	return True if ( dist >= info.sz_hand ) else False


# il blocco e' accessibile?
#  elem : ( 'name', Pose )
def block_is_accessible( elem_b ):
	global frame_block_red

	for elem_r in frame_block_red.values():
		dx = abs( elem_b[1].position.y - elem_r.position.y )
		dz = abs( elem_b[1].position.x - elem_r.position.x )
		dy = elem_b[1].position.z - elem_r.position.z 
		
		if info.use_verbose:
			rospy.loginfo( " [task manager] (%s) dx=%f | dy=%f | dz=%f | sovrapposizione: %s", "block_is_accessible", dx, dy, dz, str(dx < info.sz_cube and dz < info.sz_cube and dy < 0) )
		
		if (dx < info.sz_cube and dz < info.sz_cube and dy < 0):
			return False

	return True


# distanza tra un blocco e la mano dell'operatore
#  elem : ( 'name', Pose )
def dist_block_hand( elem ):
	# cerca la mano più vicina al blocco considerato
	frame_human_hand = get_nearest_hand_pose( elem[1] )
	
	#  calcola la distanza tra l'origine della mano e quella del blocco
	P = [ elem[1].position.y, elem[1].position.x ]
	O = [ frame_human_hand.position.y, frame_human_hand.position.x ]
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) %s[%f, %f] TABLE[%f, %f] | dist: %f", "dist_block_hand", elem[0], P[0], P[1], O[0], O[1], math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 ) )
	
	return math.sqrt( (P[0]-O[0])**2 + (P[1]-O[1])**2 )


# Posizionamento del blocco al centro
#  elem : Pose
def check_block_is_at_center( elem ):
	O = Pose()
	O.position.x = info.sz_table[0]/2
	O.position.y = 0
	O.position.z = 0
	
	dist = distance_between( elem, O )
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) [%f, %f, %f] | distance: %f | limit: %f | answer: %s", "check_block_is_at_center", O.position.x, O.position.y, O.position.z, dist, info.center_dispacement, str( dist < info.center_dispacement ) )
	
	return ( dist < info.center_dispacement )


# stampa a video i dati del task
# task_name : String
# task : command 
def print_task( task_name, task ):
	if task is not None:
		rospy.loginfo( " [task manager] TASK [%s] arm(%s) pos(%s) cube(%s)", task_name, task.arm, task.pos, task.cube )
	else:
		rospy.loginfo( " [task manager] TASK [%s] is None.", task_name )



## --------------------------- TASK MANAGER

# ciclo di funzionamento del task manager
# continua fin quando ci sono blocchi blu da spostare, poi chiudi
def task_manager_body():
	global task_arm_left, task_arm_right, frame_block_red, miss
	
	task_manager_frequency = rospy.Rate(info.task_manager_freq)
	
	while( len(frame_block_blue) > 0 and not rospy.is_shutdown() ):
		# ottieni un compito per il braccio sinistro se possibile
		task_arm_left = get_task_arm_left()
		print_task( "task_arm_left", task_arm_left )
		
		# ottieni un compito per il braccio destro se possibile
		task_arm_right = get_task_arm_right()
		print_task( "task_arm_right", task_arm_right )
		
		# esegui i comandi 
		if( task_arm_left != None or task_arm_right != None ):
			task_execute_wrapper(task_arm_right, task_arm_left)
		else:
			# attendi che la situazione cambi
			rospy.loginfo( " [task manager] Nothing to do. Waiting..." )
			task_manager_frequency.sleep()
	
	if rospy.is_shutdown():
		rospy.loginfo( " [task manager] shutdown signal received." )
	else:
		rospy.loginfo( " [task manager] Job done. (misses: %d)", miss )


# ottieni un compito per il braccio destro
def get_task_arm_right():
	global frame_block_blue, frame_block_red, frame_goal_red, block_at_center
	
	# controlla che non ci siano già blocchi al centro
	if block_at_center is not None:
		return None
	
	# considera la mano più vicina
	frame_human_hand = get_nearest_hand_pose( frame_goal_red )
	
	# non avviare la procedura se la lista è vuota. 
	if( len(frame_block_blue) == 0 ):
		return None
		
	hand_on_right = block_on_right_table(( "hand", frame_human_hand ))
	
	# posizione dei box blu
	bluebox = frame_block_blue.items()
	
	# escludi tutti i blocchi sul lato destro del tavolo 
	bluebox = list(filter( block_on_right_table, bluebox ))
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) blue blocks to evaluate: %s", "get_task_arm_right", str(dict(bluebox).keys()) )
	
	# se la lista contiene un solo elemento, salta direttamente a dopo la 
	# procedura di ricerca del blocco. 
	
	if ( len(bluebox) > 1 ):
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
		# rospy.loginfo( " [task manager] (%s) found: %s", "get_task_arm_right", str(dict(bluebox).keys()) )
		# prendi il primo blocco della lista e ritorna il task
		winner = list(bluebox)[0][0]
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
	global frame_block_blue, frame_block_red, frame_goal_blue, block_at_center
	
	# non avviare la procedura se la lista è vuota. 
	if( len(frame_block_blue) == 0 ):
		return None
	
	# considera la mano più vicina
	frame_human_hand = get_nearest_hand_pose( frame_goal_blue )
	hand_on_left = block_on_left_table(( "hand", frame_human_hand ))
	
	# posizione dei box blu
	bluebox = frame_block_blue.items()
	
	# escludi tutti i blocchi sul lato sinistro del tavolo 
	bluebox = list(filter( block_on_left_table, bluebox ))
	
	if info.use_verbose:
		rospy.loginfo( " [task manager] (%s) blue blocks to evaluate: %s", "get_task_arm_left", str(dict(bluebox).keys()) )
	
	# se la lista contiene un solo elemento, salta direttamente a dopo la 
	# procedura di ricerca del blocco. 
	
	winner = None
	
	if ( len(bluebox) > 1 ):
		# escludi tutti i blocchi troppo vicini alla mano dell'operatore
		if( len(bluebox) > 0 ):
			bluebox = list(filter( block_not_near_to_hand, bluebox ))
		
		# escludi i blocchi non accessibili
		if( len(bluebox) > 0 and len(frame_block_red) > 0 ):
			# redbox  = frame_block_red.items()
			bluebox = list(filter( block_is_accessible, bluebox ))
		
		# ordina i blocchi in base a quello più lontano dalla mano
		# ordina solo se la len è maggiore di 1
		if ( len(bluebox) >= 1 and ( block_at_center is not None ) and ( ( block_at_center, frame_block_blue[block_at_center] ) in list(bluebox) ) ):
			# prendi il blocco centrale
			winner = block_at_center
			rospy.loginfo( " [task manager] (%s) forcing block at centre: %s", "get_task_arm_left", block_at_center )
		elif ( len(bluebox) > 1 and hand_on_left ):
			bluebox.sort(key=dist_block_hand, reverse=True)
			# prendi il primo blocco della lista e ritorna il task
			winner = list(bluebox)[0][0]
			rospy.loginfo( " [task manager] (%s) winner: %s", "get_task_arm_left", winner )
	
	# chi sarà il blocco adatto?
	if( winner is None and len(bluebox) == 1 ):
		winner = list(bluebox)[0][0]
	if ( winner is not None ):
		# rospy.loginfo( " [task manager] (%s) found: %s", "get_task_arm_left", str(dict(bluebox).keys()) )
		left_task = commandRequest()
		left_task.arm = "left"
		left_task.pos = "box"
		left_task.cube = winner
		return left_task
	else:
		# nessun blocco compatibile
		return None


# decidi se usare l'esecuzione parallela o quella sequenziale
def task_execute_wrapper( task_right, task_left ):
	global frame_block_blue
	
	if ( task_right is not None ):
		label_A = frame_block_blue[ task_right.cube ]
	if ( task_left is not None ):
		label_B = frame_block_blue[ task_left.cube ]
	
	if ( ( task_right is None ) and ( task_left is None ) ):
		# nulla da fare, entrambi a None
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


# esegui i task
# ritorna True se la routine ha avuto successo, false altrimenti
def task_execute( task_right, task_left ):
	global srv_controller, srv_baxter_home, frame_block_blue, miss, block_at_center
	
	rospy.loginfo( " [task manager] sending tasks to the controller..." )
	
	right_at_home = False
	left_at_home = False
	left_box = ""
	
	# Invio del right_task al controller
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
	
	# Invio del left_task al controller
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
	
	# chiudi il controller se entrambi i task falliscono
	if right_at_home and left_at_home:
		return False
	
	# attendi che Baxter inizi a muoversi
	baxter_is_moving = False
	rospy.loginfo( " [task manager] waiting for BAXTER to start movement..." )
	while not baxter_is_moving:
		baxter_is_moving = not ( ( srv_baxter_home( arm="right" ) ).at_home and ( srv_baxter_home( arm="left" ) ).at_home )
	rospy.loginfo( " [task manager] BAXTER is moving; waiting for the tasks..." )
	
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
	
	baxter_is_moving = False
	
	# verifica la nuova posizione del blocco blu
	if task_left is not None:
		rospy.loginfo( " [task manager] checking if %s has reached the goal... ", left_box )
		if block_in_blue_goal( left_box ):
			# elimina il blocco dalla raccolta
			rospy.loginfo( " [task manager] -> BLUE block %s in goal", left_box )
			
			# verifica che il blocco che ho appena mosso fosse al centro
			if (block_at_center is not None) and ( block_at_center == left_box ):
				block_at_center = None
			
			# elimina il blocco dalla lista
			del frame_block_blue[ left_box ]
			
		else:
			miss = miss + 1
			rospy.loginfo( " [task manager] -> BLUE block %s: failed. Retry. (misses: %d)", left_box, miss )
	
	# verifica che il blocco sia al centro
	if task_right is not None:
		if check_block_is_at_center( frame_block_blue[task_right.cube] ):
			# il blocco si trova al centro
			block_at_center = task_right.cube
		else:
			miss = miss + 1
			rospy.loginfo( " [task manager] -> BLUE block %s: failed. Retry. (misses: %d)", task_right.cube, miss )
	
	rospy.loginfo( " [task manager] task execution ... done" )
	
	return True



## --------------------------- NODO

def on_shutdown_msg():
    rospy.loginfo( " [task manager] offline" )


def main():
    global srv_controller, msg_unity_tf, srv_baxter_home, update_counter
    global frame_block_blue, frame_block_red
    
    print( "progetto SKYNET avviato." )
    
    # node init
    rospy.init_node( "task_manager" )
    rospy.on_shutdown( on_shutdown_msg )
    
    # subscribe: unity_tf o tf a seconda delle impostazioni
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
    
    # attenti il primo aggiornamento
    while update_counter < 1:
        (rospy.Rate(1)).sleep()
    
    labels = list(frame_block_blue.keys())
    for key in labels:
        if block_in_blue_goal( key ):
            rospy.loginfo( " [task manager] -> BLUE block %s in goal", key )
            del frame_block_blue[ key ]
    
    # ciclo di funzionamento
    rospy.loginfo( " [task manager] online" )
    task_manager_body()



if __name__ == "__main__":
    main()

