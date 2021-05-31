# Applicazioni per un Robot Autonomo
Lo scopo delle scorse esercitazioni è stato quello di integrare le funzioni necessarie e sufficienti affinché un veicolo possa essere considerato autonomo. A questo punto risulta interessante aggiungere al robot un livello di intelligenza che gli permetta di sfruttare tali capacità al fine di eseguire task più articolati in maniera del tutto indipendente.
> :mag: ETIMOLOGIA  
> Prima di proseguire risulta interessante analizzare l'etimologia di _"veicolo autonomo"_ al fine di comprendere quali sono le capacità di cui un veicolo necessita affinché possa essere considerato tale.  
> **Veicolo**: dal lat. _vehicŭlum_, der. di _vehĕre_ «trasportare». Identifica un mezzo che trasporti qualcosa (o qualcuno), o in ogni caso se stesso, da un punto dello spazio a l'altro: un oggetto dotato dunque di un sistema di locomozione (attuatori ed eventuali organi di trasmissione) che gli permetta di spostarsi da un punto A a un punto B.  
> **Autonomo**: dal gr. _αὐτόνομος_, a cui si può far corrispondere la locuzione «che si governa da sé». Associato al veicolo, si richiede che questo sia in grado non solo di azionare il relativo sistema di locomozione, ma che sia inoltre dotato di sensori che gli permettano di localizzarsi all'interno dell'ambiente entro cui si muove.

È possibile creare un nuovo package (`turtlebot3_app`) che contenga gli esempi di applicazioni autonome per il Turtlebot 3.
```bash
$ catkin_create_pkg turtlebot3_app rospy roscpp std_msgs geometry_msgs nav_msgs
```

## Pattugliamento
Una delle applicazioni più comuni per un robot mobile è quella di pattugliare un'area predefinita. Ciò viene fatto principalmente per scopi di sorveglianza (militare o civile) o per la ricerca di oggetti d'interesse (ordigni, persone, animali) o, ancora, per scopi di monitoraggio ambientale, raccogliendo dati di aria, acqua o altro durante il pattugliamento.  
Il pattugliamento consiste di fatto nella circolazione del robot all'interno di un'area predefinita (nota a priori o non), e può essere effettuato utilizzando diverse strategie e algoritmi: spostarsi fra una serie di _waypoints_ o effettuando spostamenti casuali (_random walk_).    
Si supponga ad esempio di voler eseguire una missione di pattugliamento sfruttando una serie di _waypoints_ distribuiti all'interno della mappa, e che si voglia visitarli in successione in maniera ciclica. Nella cartella `scripts/` del package verrà creato il file `patrol.py` riportato di seguito:
```python
#! /usr/bin/env python
# - *- coding: utf- 8 - *
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Patrol():
    def __init__(self):
        rospy.init_node('patrol')
        # ------ PARAMETERS ------
        # list of waypoint:
        # [(x1,y1,ϑ1),(x2,y2,ϑ2),(x3,y3,ϑ3)]
        self.waypoints = [( 1.6,-0.6, 0.7199),\
                          ( 1.5, 1.5, 2.7586),\
                          (-1.5, 1.5,-2.0984),\
                          (-1.5,-1.5,-0.4026)]
        # ------ ACTIONS ------
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def loop(self):
        while not rospy.is_shutdown():
            for w in self.waypoints:
                # Create Goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = w[0]
                goal.target_pose.pose.position.y = w[1]
                q = tf.transformations.quaternion_from_euler(0,0,w[2])
                goal.target_pose.pose.orientation.x = q[0]
                goal.target_pose.pose.orientation.y = q[1]
                goal.target_pose.pose.orientation.z = q[2]
                goal.target_pose.pose.orientation.w = q[3]
                # Send Goal
                self.client.send_goal(goal)
                print('Raeaching the pose: '+str(w))
                self.client.wait_for_result()
                print('Pose reached')

if __name__ == '__main__':
    p = Patrol()
    p.loop()
```

Poiché la libreria `tf` di Python non supporta ancora Python 3 lo script viene implementato utilizzando l'interprete di Python 2.7 (questo è un esempio lampante dell'interoperabilità di ROS: il programmatore può utilizzare contemporaneamente differenti linguaggi di programmazione nelle sue applicazioni, senza doversi preoccupare di farli interfacciare se non tramite ROS).  
È necessario importare inizialmente i messaggi associati all'azione implementata dal nodo `move_base`
```python
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
```
Per prima cosa definire la classe `Patrol` e nel costruttore inizializzare il nodo `patrol` contenente la lista dei _waypoints_, in cui ogni punto è contenuto in una tupla di tre elementi rappresentante la posizione cartesiana (x,y) e l'orientamento desiderato ![\vartheta](https://render.githubusercontent.com/render/math?math=%5Cvartheta). Sempre nel costruttore creare un client dell'azione `move_base` (`SimpleActionClient`) e aspettare che il server venga posto in esecuzione.  
```python
def __init__(self):
    rospy.init_node('patrol')
    # ------ PARAMETERS ------
    # list of waypoint:
    # [(x1,y1,ϑ1),(x2,y2,ϑ2),(x3,y3,ϑ3)]
    self.waypoints = [( 1.6,-0.6, 0.7199),\
                      ( 1.5, 1.5, 2.7586),\
                      (-1.5, 1.5,-2.0984),\
                      (-1.5,-1.5,-0.4026)]
    # ------ ACTIONS ------
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
```
> #### :pushpin: SCELTA DEI WAYPOINTS
> Per la scelta dei _waypoints_ è necessario stabilire le coordinate e l'orientamento dei punti all'interno della mappa. È possibile fare ricorso a Rviz per aiutarsi in tale compito e sfruttare il tasto "2D Nav Goal" per assegnare i target corrispondenti ai waypoints e visualizzae il `MoveBaseActionGoal` pubblicato sul topic `/move_base/goal`.
> ```bash
> $ rostopic echo /move_base/goal
> header: 
>   seq: 1
>   stamp: 
>     secs: 24
>     nsecs:  32000000
>   frame_id: ''
> goal_id: 
>   stamp: 
>     secs: 0
>     nsecs:         0
>   id: ''
> goal: 
>   target_pose: 
>     header: 
>       seq: 1
>       stamp: 
>         secs: 24
>         nsecs:  32000000
>       frame_id: "map"
>     pose: 
>       position: 
>         x: 1.5964217186
>         y: -0.57045352459
>         z: 0.0
>       orientation: 
>         x: 0.0
>         y: 0.0
>         z: 0.350111938055
>         w: 0.936707868458
> ---
> ```
> Nel campo `goal.target_pose.pose` è possibile consultare il sottocampo `position` per ottenere le coordinate cartesiane (x,y) del punto selezionato, e il sottocampo `orientation` per l'orientamento. L'orientamento è espresso da un _quaternione_, che è possibile trasformare nei corrispettivi _angoli di Eulero_ aprendo un interprete Python 2.7 su terminale e facendo ricorso alla funzione `tf.transformations.euler_from_quaternion`.
> ```bash
> $ python
> Python 2.7.17 (default, Apr 15 2020, 17:20:14) 
> [GCC 7.5.0] on linux2
> Type "help", "copyright", "credits" or "license" for more information.
> >>> import tf
> >>> q = tf.transformations.euler_from_quaternion([0,0,0.35,0.93])
> >>> q
> (0.0, -0.0, 0.719897052054812)

Nella funzione `loop` si cicla sui _waypoints_, inviando un `MoveBaseGoal` al server per ogni waypoints da raggiungere. L'orientamento contenuto in ogni _waypoint_ è espresso per comodità in termini dell'angolo intorno a _z_, e deve essere trasformato in un quaternione prima di essere inviato all'interno del `MoveBaseGoal`. Per la documentazione completa dei campi di `MoveBaseGoal` si consiglia di consultare la documentazione ufficiale: http://docs.ros.org/fuerte/api/move_base_msgs/html/msg/MoveBaseActionGoal.html.  
Eseguire a questo punto il Turtlebot nella sua configurazione autonoma:  
```
$ roslaunch tutlebot3_navigation turtlebot3_localization.launch
```
dunque, in un nuovo terminale, eseguire
```
$ roslaunch tutlebot3_navigation move_base.launch
```
A questo punto è possibile eseguire il nodo `patrol` digitando
```bash
$ chmod +x ~/ros_ws_src/turtlebot3_app/scripts/patrol.py
$ rosrun turtlebot3_app patrol.py
```
Provare ad aggiungere un oggetto _Odometry_ in Rviz per visualizzare il path seguito dal robot durante il pattugliamaento.

![Patrolling](img/patrolling.png)

## Robot sempre più _intelligenti_
Al fine di rendere un robot più _intelligente_ e renderlo in grado di eseguire task sempre più complessi e sofisticati, è necessario modellare ogni comportamento che il robot deve avere nelle varie situazioni e come reagire alle diverse situazioni. Il comportamento che il robot deve tenere durante il suo funzionamento può divenire molto complesso, specie se dipendente dalle interazioni con l'ambiente circostante. Per modellare tutte le possibili casistiche in cui il robot può trovarsi e schedulare correttamente i suoi cambi di comportamento in realzaione al verificarsi di un evento proveniente dal mondo esterno, risulta utile l'utilizzo di uno degli strumenti cardine dell'informatica: le **macchine a stati finiti**.  
L'idea alla base è quella che un robot si possa trovare in ogni istante in un particolare **stato** di un possibile insieme finito di questi. Ogni stato rappresenta un possibile comportamento che il robot deve tenere (ad esempio _moving_, _waiting_, *collecting_data*, ecc). Quando un robot esegue il task associato allo stato, immediatamente si sposta in uno dei possibili stati successivi in funzione del risultato con cui è stato terminato il task (ad esempio dallo stato di _waiting_ si può passare a quello di _moving_ non appena si riceve un goal da raggiungere).  
A differenza della comune interpretazione che si da a una macchina a stati, gli stati in cui può trovarsi il robot non devono essere considerati come degli _stati statici_, in cui il robot assume una certa configurazione, bensì devono rappresentare un _comportamento dinamico_ del robot, ovvero l'esecuzione di un task che potrebbe includere ad esempio il suo mobìvimento verso il punto desiderato.
Le **transizioni** che permettono il passaggio da uno stato al successivo specificano la struttura della macchina a stati, e permettono di modellare l'evoluzione comportamentale del robot nel tempo. In funzione del successo o meno dell'azione che si sta eseguendo viene generata una transizione verso un'altro stato, associando ad ognuna di esse una _etichetta_ (_label_) che permette di identificare quando quella transizione deve essere percorsa (ad esempio: _success_, _aborted_, _failed_).  

![Example of state machine](img/state_machine_01.png)

### SMACH
È possibile ricorrere al framework `smach` ("SMACH is a contraction derived from StateMACHine and pronounced like “smash”." [Bohren and Cousins, 2010]) e alla sua estensione per ROS `smach_ros`. Il core di `smach` è indipendente da ROS e consiste in una manciata di librerie scritte in Python. Questo framework, in accordo al _software design pattern composite_, esporta due principali interfacce (_State_ e _Container_), le quali permettono la costruzione di entità per la modellazione di macchine a stati gerarchiche.
#### State
Un oggetto _State_ rappresenta lo "stato di esecuzione"  per il robot e ognuno di essi specifica tutte le possibili uscite (_outcomes_) dello stato verso un altro. L'oggetto `State` deve implementare il metodo `execute` che è il responsabile dell'implementazione del comportamento che il robot deve tenere fintanto che si trova nel suddetto stato. Tale funzione deve inoltre restituire come valore di ritorno l'_outcome_ che identifica come è terminato lo stato in esecuzione. Di seguito si riporta l'esempio di implementazione per un oggetto `State`:
```python
import time
from smach import State, StateMachine

class State1(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','abort'])

    def execute(self, userdata):
        print('One')
        try:
            time.sleep(1)
            return 'success'
        except:
            return 'abort'
```
#### Container
Un _Container_ è una collezione di uno o più stati che devono essere eseguiti secondo una politica ben precisa. Il _Container_ più semplice è  `StateMachine` che permette di modellare la macchina a stati come un diagramma di flusso, in cui ogno `State` è concatenato ai successivi in funzione degli _outcomes_ generati. Un oggetto `StateMachine` è a sua volta uno `State` in quanto implementa la sua interfaccia, il che implica che anch'esso dovrà produrre un insieme di _outcomes_.

![Example of state machine](img/state_machine_02.png)

```python
# Create StateMachine
sm = StateMachine(outcomes=['succeeded', 'aborted'])
# Add States
with sm:
    StateMachine.add('S1', State1(), transitions={'success':'S2', 'abort':'aborted'})
    StateMachine.add('S2', State2(), transitions={'success':'S3', 'abort':'S1'})
    StateMachine.add('S3', State3(), transitions={'success':'succeeded', 'abort':'S1'})
```

Un altro _Container_ molto utile è `Concurrence`, la cui politica prevede l'esecuzione simultanea di tutti gli stati che contiene. Quando un _Container_ viene inizializzato è possibile definire all'interno di un dizionario Python la mappa degli _outcomes_ (`outcome_map`) che specifica le transizioni fra stati: ad ogni transizione (chiave) viene associato lo stato successivo (valore). Vengono specificate inoltre le transizioni verso gli _outcomes_ del Container in quanto anch'esso uno stato a sua volta.
Nei Container `Concurrence` è possibile inoltre definire due _callback_: `child_termination_cb` e `outcome_cb`, che specificano rispettivamente un'azione che il Container deve compiere dopo che ogni _stato figlio_ o l'ultimo di essi termina.
È inoltre possibile specificare all'interno dei Container un dizionario i variabili denominato `userdata`, il quale contiene un insieme di variabili condivise tra i vari stati.  
Segue un esempio di una semplice macchina astati implementata con `smach`.

```python
#! /usr/bin/env python3
import time
from smach import State, StateMachine

class State1(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','abort'])
    
    def execute(self, userdata):
        print('One')
        try:
            time.sleep(1)
            return 'success'
        except:
            return 'abort'
    
class State2(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','abort'])
        self.retry = True
    
    def execute(self, userdata):
        print('Two')
        try:
            time.sleep(1)
            if self.retry:
                self.retry = False
                raise Exception('RETRY')
            return 'success'
        except:
            return 'abort'

class State3(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','abort'])
    
    def execute(self, userdata):
        print('Three')
        try:
            time.sleep(1)
            return 'success'
        except:
            return 'abort'
        
if __name__ == '__main__':
    # Create StateMachine
    sm = StateMachine(outcomes=['succeeded', 'aborted'])
    # Add States
    with sm:
        StateMachine.add('S1', State1(), transitions={'success':'S2', 'abort':'aborted'})
        StateMachine.add('S2', State2(), transitions={'success':'S3', 'abort':'S1'})
        StateMachine.add('S3', State3(), transitions={'success':'succeeded', 'abort':'S1'})
    
    sm.execute()
```
Provando ad eseguire lo script precedente si ottiene un risultato simile  aquello riportato di seguito:
```bash
$ python automa.py
[ DEBUG ] : ...
[  INFO ] : State machine starting in initial state 'S1' with userdata: 
	[]
One
[  INFO ] : State machine transitioning 'S1':'success'-->'S2'
Two
[  INFO ] : State machine transitioning 'S2':'abort'-->'S1'
One
[  INFO ] : State machine transitioning 'S1':'success'-->'S2'
Two
[  INFO ] : State machine transitioning 'S2':'success'-->'S3'
Three
[  INFO ] : State machine terminating 'S3':'success':'succeeded'

```

### Patrol with SMACH
È possibile implementare il task di pattugliamento Using ```smach``` is possible to perform the patrolling behavior using a state machine. This because patrolling can be see as a flowchart where the robot drive to a waypoint to anhoter continuously. For perform the task, we only need to implement a single state corresponding to driving to a particular waypoint and define the order on which reach the target pose:
```python
#! /usr/bin/env python
# - *- coding: utf- 8 - *
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine

class Waypoint(State):
    def __init__(self, w):
        State.__init__(self,outcomes=['success','stopped'])
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        # Create Goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = w[0]
        self.goal.target_pose.pose.position.y = w[1]
        q = tf.transformations.quaternion_from_euler(0,0,w[2])
        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]

    def execute(self, userdata):
        # Send Goal
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        if self.client.get_result() == None:
            return 'stopped'
        return 'success'


class PatrolAutoma():
    def __init__(self):
        rospy.init_node('patrol')

        # ------ PARAMETERS ------
        # waypoints:
        # [(x,y,ϑ),(x,y,ϑ),(x,y,ϑ)]
        self.waypoints = [(1.6,-0.6,0.7199),(1.5,1.5,2.7586),(-1.5,1.5,-2.0984),(-1.5,-1.5,-0.4026)]
        
        self.sm = StateMachine(outcomes=['terminated'])
        with self.sm:
            for i in range(len(self.waypoints)):
                # Add states: (name, Waypoint, transitions)
                StateMachine.add('W'+str(i),Waypoint(self.waypoints[i]),transitions={'success':'W'+str((i+1)%len(self.waypoints)), 'stopped':'terminated'})

    def loop(self):
        self.sm.execute()
        
        
if __name__ == '__main__':
    pa = PatrolAutoma()
    pa.loop()
```

La classe `Waypoint` implementa l'interfaccia di `State` realizzando lo stato di _"guida verso il waypoint `w`"_. Per ogni isanza della classe, nel metodo `__init__`, viene pasato lo specifico waypoint da raggiungere, il quale verrà utilizzato per costruire il punto target come `MoveBaseGoal`. Sempre nel costruttore è instanziato un _client_ della `MoveBaseAction`. Nel metodo `execute` il `goal` viene inviato al _server_ `move_base` e, se verrà raggiunto, verrà generato  l'_outcome_ `success` come valore di uscita dello stato.  
La classe `PatrolAutoma` realizza il nodo ROS e costruisce la macchina a stati che contiene uno stato `Waypoint` per ogni punto da raggiungere: ogni qualvolta il punto corrente viene raggiunto si transita nello stato successivo che guida il robot verso il nuovo waypoint. Il meccanismo può essere ripetutto fino alla visita di tutti i waypoint e ripetuto ciclicamente all'infinito.  
Eseguire un'istanza del Turtlebot 3
```
$ roslaunch tutlebot3_navigation turtlebot3_localization.launch
```
dopodiché eseguire il `move_base` _server_
```
$ roslaunch tutlebot3_navigation move_base.launch
```
ed eseguire il nodo `patrol`
```bash
$ chmod +x ~/ros_ws_src/turtlebot3_app/scripts/patrol_automa.py
$ rosrun turtlebot3_app patrol_automa.py
```

Un altro modo per implementare il task di pattugliamento attraverso un automa a stati finiti e SMACH, è quello di utilizzare la classe `SimpleActionState` del package `smach_ros`, la quale permette di instanziare uno oggetto `State` che fa uso di una azione `actionlib`. Per uno stato di tipo `SimpleActionState` il valore degli _outcomes_ è generato in automatico a partire dai risultati dell'azione.
```python
#!/usr/bin/env python
# - *- coding: utf- 8 - *

import rospy
import tf
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolAutoma2():
    def __init__(self):
        rospy.init_node('patrol')

        # ------ PARAMETERS ------
        # waypoints:
        # [(x,y,ϑ),(x,y,ϑ),(x,y,ϑ)]
        self.waypoints = [(1.6,-0.6,0.7199),(1.5,1.5,2.7586),(-1.5,1.5,-2.0984),(-1.5,-1.5,-0.4026)]
        
        self.sm = StateMachine(['succeeded','aborted','preempted'])
        with self.sm:
            for i,w in enumerate(self.waypoints):
                # Create Goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = w[0]
                goal.target_pose.pose.position.y = w[1]
                q = tf.transformations.quaternion_from_euler(0,0,w[2])
                goal.target_pose.pose.orientation.x = q[0]
                goal.target_pose.pose.orientation.y = q[1]
                goal.target_pose.pose.orientation.z = q[2]
                goal.target_pose.pose.orientation.w = q[3]

                StateMachine.add('W'+str(i),SimpleActionState('move_base',MoveBaseAction,goal=goal),transitions={'succeeded':'W'+str((i+1)%len(self.waypoints))})
                            
    def loop(self):
        self.sm.execute()
        
if __name__ == '__main__':
    pa2 = PatrolAutoma2()
    pa2.loop()
```
