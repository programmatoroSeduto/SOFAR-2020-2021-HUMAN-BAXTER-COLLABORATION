# Human Baxter Collaboration - Real Implementation 

## Come avviare la simulazione sul robot reale

Avvio della coumunicazione col vero Baxter:

> roslaunch '/home/simone/Desktop/sofar_ws/src/human_baxter_collaboration/launch/joint_trajectory_client.launch' 

Per chiudere i gripper nel caso rimanessero aperti:

> rosrun baxter_tools tuck_arms.py -u

A questo punto, avvia

> roslaunch human_baxter_collaboration human_baxter_collaboration.launch

a quel punto, avvia l'architettura come al solito. 

## Windows - connessione di Unity

Innanzitutto, controlla che il profilo di rete sia impostato su *Pubblico*.

I sistemi di "protezione" di Windows 10 impediscono a Unity di comunicare via socket con Baxter. Per risolvere il problema, segui questi passaggi:

1. "Windows Defender Firewall con Sicurezza Avanzata"
2. finestra principale in mezzo -> anteprima -> "Proprietà Windows Defender Firewall"
3. tab "Profilo di Dominio" -> scheda "Stato" -> "Stato firewall" -> seleziona **Disattivato**
4. ora cerca "protezione firewall e della rete"
5. disattiva il firewall sulla rete pubblica

## Windows - trovare il mio IP

Per avviare gli script su Baxter è necessario sapere il proprio indirizzo IP. 

In Windows 10, fai così:

1. clic sulla connessione wifi nella barra delle notifiche
2. stato -> stato della rete -> tasto "Proprietà"
3. sezione "Proprietà" -> trovi il tuo indirizzo in "Indirizzo IPv4"

## Baxter reale Vs. Simulazione 

Purtroppo il robot simulato è diverso dal robot reale sotto molti aspetti. Non assicuro di ricordare tutte le modifiche necessarie. Elenco qui di seguito solo le principali: 

- il gripper viene controllato dallo script fornito dal proessore, ma eventualmente è possibile creare un nodo che gestisca il gripper quando serve e che prenda i comandi dal *controller_baxter*

- Il braccio non torna alla posizione iniziale al termine del movimento, ma rimane nella posizione data. Questo si risolve apportando modifiche al *controller_baxter*. 
  La seguente riga va commentata per mantenere una coerenza con il robot reale. Altrimenti quello in Unity torna alla rest position, mentre il robot reale rimane lì.

    ```c#
    // in Scripts/BaxterController.cs
    // riga 408
    GoToRestPosition(response.arm);
    ```

- La posizione iniziale del Baxter reale non coincide con quella del Baxter simulato (no comment...). Vedi *Scripts/BaxterController.cs* da Unity. Anche qui, una modifica al *controller_baxter* è necessaria. 

    ```c#
    // righe 242, 269
    // prima
    float[] target = { -30f, -70f, 0f, 99f, 0f, 43f, 0f };
    float[] target = { 30.0f, -70.0f, 0f, 99.0f, 0f, 43.0f, 0f };

    // dopo
    float[] target = { -0.5f, -57f, -68f, 110f, 38f, 59f, -29f };
    float[] target = { 0.5f, -57f, 68f, 110f, -38f, 59f, 29f };

    ```

- il Baxter in laboratorio è terribilmente più veloce di quello simulato. Per ridurre la velocità del Baxter reale, usa lo script nel package ROS *\human_baxter_collaboration\scripts\joint_trajectory_client.py*.

    ```python
    # da riga 86
    def trajectory_callback(msg):
        if(limb == msg.arm):
            #rospy.sleep(5)
            arm = msg.arm
                
            n = len(msg.trajectory)
            close_gripper_idx = 1
            open_gripper_idx = 4
            wait_before_opening =  1
            
            for i in range(len(msg.trajectory)):
                
                traj = Trajectory(arm)
                # Start with open gripper
                if i == 0:
                    traj.open_gripper()
                    
                rospy.on_shutdown(traj.stop)
                # Command Current Joint Positions first
                limb_interface = baxter_interface.limb.Limb(arm)
                    
                t = 1
                for point in msg.trajectory[i].joint_trajectory.points:
                        p = point.positions
                        traj.add_point(p, t)
                        t += 2.5 # QUESTO...
                
                traj.start()
                traj.wait(t + 1) # ... E QUESTO
                # Close gripper on grasping
                if i == close_gripper_idx:
                    rospy.sleep(1)
                    traj.close_gripper()
                # Reopen gripper on release
                elif i == open_gripper_idx:
                    rospy.sleep(wait_before_opening)
                    traj.open_gripper()
                    rospy.sleep(1)

            print("Joint Trajectory Action Complete")
    ```

    Lo stesso script server per gestire i gripper:

    ```python
    # da riga 86
    n = len(msg.trajectory)
    close_gripper_idx = 1 # Numero di traiettorie date dopo il quale aprire il gripper
    open_gripper_idx = 4 # Stessa cosa, per chiudere il gripper
    wait_before_opening =  1
    ```

- Il nodo *baxter_at_home* potrebbe essere inutile, perchè attualmente il gripper non ha bisogno di tornare alla posizione di partenza. Due soluzioni sono possibili in questo caso: 1)modificare il *controller_baxter* in modo che il robot torni sempre alla posizione di partenza al termine del task 2)proseguire il movimento anche senza tornare a casa, il che implica delle modifiche al feedback del *task_manager*, usando le tf al posto della rest position. Modifica semplice, a patto di trovare il frame che si muove quando il braccio si muove. 