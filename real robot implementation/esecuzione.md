# COme avviare la simulazione sul robot reale

Avvio di Baxter:

> roslaunch '/home/simone/Desktop/sofar_ws/src/human_baxter_collaboration/launch/joint_trajectory_client.launch' 

pr chiudere i gripper nel caso rimanessero aperti:

> rosrun baxter_tools tuck_arms.py -u

A questo punto, avvia

> roslaunch human_baxter_collaboration human_baxter_collaboration.launch

a quel punto, avvia l'architettura come al solito. 
