# Note sul task manager

# Eseguire il task manager e il progetto

Avviare ROS e il launch file 

```
roscore &
roslaunch human_baxter_collaboration human_baxter_collaboration.launch &
```

Su windows, avviare il pacchetto `MMILauncher.exe`

Sempre su windows, fai partire Unity e lancia la simulazione. Dovresti vedere sulla consolle di ROS un messaggio del tipo

```bash
ROS-Unity Handshake received, will connect to 192.168.65.2:5005
```

lancia la simulazione, "play"→ start simulation

A questo punto puoi far partire il controller e il task manager:

```bash
rosrun controller_baxter baxter_at_home > /dev/null &
rosrun controller_baxter controller_baxter > /dev/null & 

rosrun controller_baxter task_manager.py
```

# Ipotesi di funzionamento

## Movimenti dell'operatore

- L'operatore si muove molto più velocemente di Baxter
- L'operatore usa entrambe le mani
- La mano dell'operatore potrebbe anche stare ferma sul tavolo; in questo caso, Baxter la eviterebbe programmando il movimento in modo tale da mantenersi più distante possibile dalla mano
- l'operatore non può prendere i blocchi blu e spostarli, nè direttamente e nè indirettamente
- l'operatore non può ostruire i blocchi. Meglio non avere questo comportamento, dato che il tempo tra l'invio del messaggio al controller e il movimento è troppo lungo
- l'operatore usa una sola mano alla volta

## Cubetti

- Baxter può muovere solo i blocchi blu, senza toccare quelli rossi
- non possono rotolare al di fuori del tavolo
- Certe volte il grasp dei cubetti può fallire, ad esempio quando il gripper non viene orientato perfettamente.

# Il task manager

## Messaggi e configurazione

Ogni task è rappresentato da un servizio di tipo `controller_baxter.command`, strutturato come segue:

```yaml
# controller_baxter/command.srv
# arm: "left", "right"
string arm
# pos: "box", "center"
string pos
# cube: E, M, C, G, I
string cube
---
bool ok
```

Per il funzionamento della fase di esecuzione, è necessario anche un altro servizio di tipo `controller_baxter.at_home`, strutturato così:

```yaml
# controller_baxter/at_home.srv
# arm: "left", "right"
string arm
---
bool at_home
```

Parametri di funzionamento: in sim_infos.py . Ovviamente, il buonn funzionamento del programma dipende dalla scelta opportuna di questi valori. Questo file serve anche per raccogliere in un punto buona parte dell'hard coding necessario per questo progetto. 

```python
## --------------------------- DIMENSIONI OGGETTI DELLA SIMULAZIONE

# lato del cubetto (tutti i lati uguali)
sz_cube = 0.05

# dimensioni x e y del goal (x, y, z)
sz_goal = [ 0.14, 0.01, 0.221 ]

# dimensioni del tavolo (x, y, z)
sz_table = [ 2, 0.8, 1 ]

# dimensioni della mano (soglia di allarme)
#    raggio di una circonferenza
sz_hand = 0.3

## --------------------------- NOMI DI TOPIC E SERVIZI

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

## --------------------------- ALTRI PARAMETRI

# frequenza del task manager
#    quante volte viene rieseguito il calcolo dei task in stato di attesa
task_manager_freq = 2

# frequenza del log da Unity
#    ogni quanti messaggi stampare un'informazione sul topic unity_tf
log_freq = 250

# frequenza del log completo da Unity
#    ogni quanti cicli stampare a video lo stato integrale
log_freq_extended = 1000000

# distanza minima che i due blocchi devono avere per l'esecuzione parallela
#    0.5 probabilmente è un po' esagerato come valore... 0.2 è tropo piccolo però
minimum_distance_parallel = 0.5
```

## Funzionamento generale

Il task manager funziona in 3 step, tutti raccolti in `task_manager_body()`:

- assegnazione di un compito al braccio destro, se possibile
- assegnazione di un compito al braccio sinistro, se possibile
- Esecuzione dei task, parallela o sequenziale

Se non c'è nulla da eseguire, ci sono due opzioni. Primo: rimangono blocchi blu che però non sono accessibili; Baxter rimane in attesa dell'azione umana. Secondo: non ci sono più blocchi blu; il task manager termina. 

# Ricerca del prossimo blocco da prendere

Il nodo esegue un controllo statico sulle informazioni dall'ambiente simulativo valutando una serie di condizioni. La ricerca procede in modo simile per entrambe le braccia: si parte dall'insieme dei blocchi, e passo dopo passo, condizione dopo condizione, si arriva all'insieme finale dei blocchi che si possono prendere. 

Il programma può essere facilmente modificato per adattarsi a robot anche con sensoristica migliore. Purtroppo il modello fornito ha tempi di reazione estremamente lenti, dunque poco si può fare per le collisioni e la collaborazione, se non prevenire staticamente intoppi.

Nel procedimento viene considerata la mano più vicina al goal come mano di riferimento. L'altra viene trascurata. (vedi ipotesi sull'operatore)

Procedimento:

- anzitutto, selezione dei blocchi che appartendono ad un certo lato del tavolo. Ad esempio, il braccio destro può prendere solo i blocchi a destra.
- escludi tutti i blocchi vicini oltre la soglia di sicurezza alla mano dell'operatore (vedi file configurazione)
- escludi tutti i blocchi non accessibili, cioè al di sotto di altri blocchi rossi.
- ordina i blocchi da quello più lontano dalla mano dell'operatore a quello più vicino
- viene preso il primo ... se la lista non è vuota

Il braccio destro ha sempre il comando `center`, mentre il braccio sinistro ha sempre il comando `box`. Importante notare che il controller, quando viene richiesto il passa-mano dal task manager, dopo aver ricevuuto il comando `center` posiziona il blocco *un pochino più a sinistra* del centro del tavolo, in modo che il task manager riconosca il blocco "centrato" come appartenente alla zona sinistra, qundi pronto per essere trasportato nel goal.

# Esecuzione

Due funzioni: `task_execute_wrapper()` che decide se prendere i blocchi in maniera sequenziale oppure parallela in base ai task forniti e alla distanza tra i blocchi da prendere (questo permette di prevenire che i due gripper si vadano a toccare); e `tak_execution()` la funzione principale per l'esecuzione. Il `task_manager_body()`, dopo aver completato la decisione dei blocchi da prendere, chiama `task_execution_wrapper()` la quale sceglie come eseguire i due task. Modalità:

- *only one arm*: è stato assegnato un solo task ad un solo braccio, l'altro è idle
- *parallel execution*: le braccia si muovono indipendentemente
- *sequential execution*: prima il destro, e solo dopo il sinistro.

La soglia di emergenza sta nel file sim_infos.py che raccoglie tutte le informazioni usate durante l'esecuzione (e la maggior parte del'hard coding, messo qui proprio per non averlo in giro per il codice).

Fatto questo, viene avviata la routine `task_execution()` che funziona in in questa maniera:

- invio dei messaggi al controller
- attendi che Baxter abbia iniziato a muoversi (basta anche un solo braccio) usando *baxter_at_home*
- attendi che Baxter abbia smesso di muoversi, sempre usando *baxter_at_home*
- in caso sia stato dato un task_left da eseguire, verifica se il blocco ha raggiunto il goal, e in caso eliminalo dall'insieme dei blocchi da sistemare.

Nota: il controller potrebbe ritornare false, perciò sono state adottate strategie per gestire questo possibile errore:

```python
# Invio del right_task al controller
	if task_left is not None:
		left_box = task_left.cube
		res = srv_controller( task_left )
		if not res.ok:
			rospy.loginfo( " [task manager] CONTROLLER FAILURE in sending left task" )
			miss = miss + 1
			# se non c'è altro da eseguire, chiudi
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
```

# Sviluppi futuri

- Incrementare il livello di parallelismo usando una action per interagire con Baxter anzichè un servizio. In questo modo, il controller può già iniziare a cercare il prossimo blocco senza dover per forza attendere che tutto il movimento sia stato compiuto prima di iniziare il prossimo.

# Un esempio di funzionamento

... mettere qualche filmato su Google Drive? 

In questo esempio, che usa la configurazione standard fornita dai prof, 

- Il robot rimuove il primo blocco a sinistra, l'unico che può rimuovere
- mentre lo sta rimuovendo, l'umano inizia a muoversi
- collaborazione.

Consolle del task manager: 

```markdown
[INFO] [1623145198.326408]:  [task manager] subscribe: unity_tf ...
[INFO] [1623145198.328779]:  [task manager] subscribe: unity_tf ... OK!
[INFO] [1623145198.330122]:  [task manager] service: controller_server ...
[INFO] [1623145198.333360]:  [task manager] service: controller_server ... OK!
[INFO] [1623145198.334270]:  [task manager] service: baxter_at_home_server ...
[INFO] [1623145198.336302]:  [task manager] service: baxter_at_home_server ... OK!
[INFO] [1623145199.338251]:  [task manager] online
[INFO] [1623145199.339606]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(C)
[INFO] [1623145199.340587]:  [task manager] TASK [task_arm_right] is None.
[INFO] [1623145199.341583]:  [task manager] execution: only left
[INFO] [1623145199.342434]:  [task manager] sending tasks to the controller...
[INFO] [1623145204.958502]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145206.959363]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145213.351751]:  [task manager] update no. 250
[INFO] [1623145213.353477]:  [task manager]     frame_block_blue len:5
[INFO] [1623145213.354811]:  [task manager]     frame_block_red len:6
[INFO] [1623145228.062749]:  [task manager] checking if C has reached the goal... 
[INFO] [1623145228.063719]:  [task manager] -> BLUE block C in goal
[INFO] [1623145228.064620]:  [task manager] task execution ... done
[INFO] [1623145228.065530]:  [task manager] TASK [task_arm_left] is None.
[INFO] [1623145228.066511]:  [task manager] TASK [task_arm_right] arm(right) pos(center) cube(M)
[INFO] [1623145228.067393]:  [task manager] execution: only right
[INFO] [1623145228.068295]:  [task manager] sending tasks to the controller...
[INFO] [1623145228.192472]:  [task manager] -> RED block A in goal
[INFO] [1623145228.664706]:  [task manager] update no. 500
[INFO] [1623145228.666126]:  [task manager]     frame_block_blue len:4
[INFO] [1623145228.667255]:  [task manager]     frame_block_red len:5
[INFO] [1623145233.658421]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145235.948208]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145239.016643]:  [task manager] -> RED block L in goal
[INFO] [1623145243.193440]:  [task manager] update no. 750
[INFO] [1623145243.195093]:  [task manager]     frame_block_blue len:4
[INFO] [1623145243.196219]:  [task manager]     frame_block_red len:4
[INFO] [1623145244.129089]:  [task manager] -> RED block H in goal
[INFO] [1623145255.520450]:  [task manager] -> RED block D in goal
[INFO] [1623145257.429489]:  [task manager] update no. 1000
[INFO] [1623145257.430990]:  [task manager]     frame_block_blue len:4
[INFO] [1623145257.432023]:  [task manager]     frame_block_red len:2
[INFO] [1623145260.905359]:  [task manager] -> RED block F in goal
[INFO] [1623145261.051764]:  [task manager] task execution ... done
[INFO] [1623145261.053363]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(G)
[INFO] [1623145261.054393]:  [task manager] TASK [task_arm_right] arm(right) pos(center) cube(E)
[INFO] [1623145261.055288]:  [task manager] execution: parallel
[INFO] [1623145261.056346]:  [task manager] sending tasks to the controller...
[INFO] [1623145271.559175]:  [task manager] update no. 1250
[INFO] [1623145271.560769]:  [task manager]     frame_block_blue len:4
[INFO] [1623145271.561940]:  [task manager]     frame_block_red len:1
[INFO] [1623145271.724118]:  [task manager] -> RED block B in goal
[INFO] [1623145271.726099]:  [task manager] human job done.
[INFO] [1623145272.258488]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145272.262995]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145286.443277]:  [task manager] update no. 1500
[INFO] [1623145286.444868]:  [task manager]     frame_block_blue len:4
[INFO] [1623145286.446054]:  [task manager]     frame_block_red len:0
[INFO] [1623145291.066071]:  [task manager] checking if G has reached the goal... 
[INFO] [1623145291.067420]:  [task manager] -> BLUE block G in goal
[INFO] [1623145291.068501]:  [task manager] task execution ... done
[INFO] [1623145291.069473]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(I)
[INFO] [1623145291.070335]:  [task manager] TASK [task_arm_right] arm(right) pos(center) cube(E)
[INFO] [1623145291.071167]:  [task manager] execution: parallel
[INFO] [1623145291.072004]:  [task manager] sending tasks to the controller...
[INFO] [1623145301.683733]:  [task manager] update no. 1750
[INFO] [1623145301.685221]:  [task manager]     frame_block_blue len:3
[INFO] [1623145301.686416]:  [task manager]     frame_block_red len:0
[INFO] [1623145302.258450]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145302.261568]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145316.407855]:  [task manager] update no. 2000
[INFO] [1623145316.409617]:  [task manager]     frame_block_blue len:3
[INFO] [1623145316.410874]:  [task manager]     frame_block_red len:0
[INFO] [1623145325.164793]:  [task manager] checking if I has reached the goal... 
[INFO] [1623145325.165871]:  [task manager] -> BLUE block I in goal
[INFO] [1623145325.166937]:  [task manager] task execution ... done
[INFO] [1623145325.168214]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(E)
[INFO] [1623145325.169407]:  [task manager] TASK [task_arm_right] is None.
[INFO] [1623145325.170534]:  [task manager] execution: only left
[INFO] [1623145325.171749]:  [task manager] sending tasks to the controller...
[INFO] [1623145330.758424]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145330.985913]:  [task manager] update no. 2250
[INFO] [1623145330.988080]:  [task manager]     frame_block_blue len:2
[INFO] [1623145330.990261]:  [task manager]     frame_block_red len:0
[INFO] [1623145333.124952]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145345.378936]:  [task manager] update no. 2500
[INFO] [1623145345.380706]:  [task manager]     frame_block_blue len:2
[INFO] [1623145345.381889]:  [task manager]     frame_block_red len:0
[INFO] [1623145355.129223]:  [task manager] checking if E has reached the goal... 
[INFO] [1623145355.130756]:  [task manager] -> BLUE block E in goal
[INFO] [1623145355.131762]:  [task manager] task execution ... done
[INFO] [1623145355.132767]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(M)
[INFO] [1623145355.133693]:  [task manager] TASK [task_arm_right] is None.
[INFO] [1623145355.134616]:  [task manager] execution: only left
[INFO] [1623145355.135460]:  [task manager] sending tasks to the controller...
[INFO] [1623145360.246150]:  [task manager] update no. 2750
[INFO] [1623145360.247819]:  [task manager]     frame_block_blue len:1
[INFO] [1623145360.249587]:  [task manager]     frame_block_red len:0
[INFO] [1623145360.658489]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145363.135172]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145375.089120]:  [task manager] update no. 3000
[INFO] [1623145375.090870]:  [task manager]     frame_block_blue len:1
[INFO] [1623145375.092185]:  [task manager]     frame_block_red len:0
[INFO] [1623145390.095815]:  [task manager] update no. 3250
[INFO] [1623145390.097659]:  [task manager]     frame_block_blue len:1
[INFO] [1623145390.099088]:  [task manager]     frame_block_red len:0
[INFO] [1623145399.339040]:  [task manager] checking if M has reached the goal... 
[INFO] [1623145399.340544]:  [task manager] -> BLUE block M: failed. Retry. (misses: 1)
[INFO] [1623145399.341900]:  [task manager] task execution ... done
[INFO] [1623145399.343118]:  [task manager] TASK [task_arm_left] arm(left) pos(box) cube(M)
[INFO] [1623145399.344182]:  [task manager] TASK [task_arm_right] is None.
[INFO] [1623145399.345285]:  [task manager] execution: only left
[INFO] [1623145399.346471]:  [task manager] sending tasks to the controller...
[INFO] [1623145404.958409]:  [task manager] waiting for BAXTER to start movement...
[INFO] [1623145405.231847]:  [task manager] update no. 3500
[INFO] [1623145405.233677]:  [task manager]     frame_block_blue len:1
[INFO] [1623145405.235916]:  [task manager]     frame_block_red len:0
[INFO] [1623145407.288180]:  [task manager] BAXTER is moving; waiting for the tasks...
[INFO] [1623145420.047421]:  [task manager] update no. 3750
[INFO] [1623145420.049158]:  [task manager]     frame_block_blue len:1
[INFO] [1623145420.050721]:  [task manager]     frame_block_red len:0
[INFO] [1623145429.491123]:  [task manager] checking if M has reached the goal... 
[INFO] [1623145429.492333]:  [task manager] -> BLUE block M in goal
[INFO] [1623145429.493296]:  [task manager] task execution ... done
[INFO] [1623145429.494226]:  [task manager] Job done. (misses: 1)
[INFO] [1623145429.495401]:  [task manager] offline.
```