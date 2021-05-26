# TURTLEBOT 3
Nella pagina del wiki (http://robots.ros.org/) è possibile trovare una vasta collezione di robot (terrestri, aerei, marini) che supportano ROS. 
Alcuni di questi includono robot personalizzati rilasciati pubblicamente dagli sviluppatori ed è un elenco notevole considerando che un singolo sistema supporta robot così diversi. Oltre 200 robot reali sviluppati con ROS sono ad oggi commercializzati sul mercato internazionale. Fra questi, uno dei più noti è sicuramente il **TurtleBot**.  
Il **Turtlebot 3** è un robot compatto molto usato in ambito accademico e di ricerca grazie alla sua architettura _open-source_ e alla facilità di riprogrammazione delle sue componenti. Grazie anche alle numerose integrazioni di cui può essere equipaggiato, il Turtlebot si presta anche molto bene alla prototipazione rapida (soprattuto per la validazione di nuovi algoritmi). Il Turtlebot 3 ha rilasciato 3 modelli ufficiali: Burger, Waffle e Waffle Pi.  
![TurtleBot 3 models](img/turtlebot_01.png)
Hardware e software sono entrambi open-source (compresi i file `*.stl` del modello 3D) e distribuiti su cloud da parte della _ROBOTIS Co. Ltd_, ciò permette agli utenti di modificare e ersonalizzare ogni parte del robot secondo le proprie esigenze.

## Hardware
Il Turtlebot 3 è un differential-drive basato su due motori _Dynamixel_ per l'attuazione delle due ruote principali, un computer di bordo (_single-board computer_) per l'esecuzione di ROS (Raspbarry Pi o Intel Joule), una scheda a di controllo (_embedded board_) basata su microcontrollore _Cortex-M7_; è possibile inoltre equipaggiarlo di una serie di sensori, tra cui una IMU, una _depth-camera_ per la ricostruzione 3D dell'ambiente circostante, LiDaR, ecc.
![TurtleBot 3 equipment](img/turtlebot_02.png)

ROS richiede un sistema operativo per essere eseguito, come potrebbe essere una distribuzione Linux; tuttavia è noto che i sistemi operativi convenzionali non sono in grado di garantire le prestazioni real-time necessarie per alcuni tipi di sensori e attuatori; per far fronte a i _single-board computer_ sono in genere affiancati da moduli a microcontrollore per eseguire tali operazioni a basso livello.  
Il TurtleBot3 fa uso di microcontrollori della famiglia ARM, serie Cortex-M7montati sulla scheda **OpenCR** (_Open-source Control Module for ROS_) a cui è demandata l'esecuzione dei task a basso livello.  
La scheda OpenCR utilizza un chip _STM32F746_ come MCU principale, e fornisce un'interfaccia compatibile con i prodotti _Arduino UNO_, ciò garantisce la presenza e la compatibilità di una vasta gamma di librerie, codici sorgente e schede di espansione, sviluppate per l'ambiente di sviluppo di Arduino.  
La scheda embedded OpenCR supporta le principali interfacce di comunicazione: UART, I2C, SPI, CAN, TTL, RS485, e include un chip _MPU925010_, il quale integra al suo interno un giroscopio a tre assi, un accelerometro a tre assi e un magnetometro a tre assi; ciò permette di avere già a disposizione un IMU necessaria in un gran numero di applicazioni base.  
OpenCR fornisce un firmware atto a comunicare con le varie periferiche della scheda, il quale può essere liberamente scaricato e caricato sulla scheda attraverso l'IDE di Arduino.

## Software
Il software a bordo del Turtlebot 3 suddiviso nel seguente modo:
- un firmware usato come controllore fisico, in esecuzione sulla scheda embedded;
- 4 package ROS eseguibili sul computer di bordo.

Il firmware in esecuzione sulla scheda OpenCR è noto come `turtlebot3_core` e viene caricato sulla scheda embedded che si trova a bordo del Turtlebot. Tale firmware ha il compito di gestire gestire e attuare i motori, leggere i dati degli encoder e degli altri sensori a bordo (IMU) e fornire una stima odometrica della posizione del veicolo. La comunicazione tra la scheda a microcontrollore e il computer di bordo sul quale è in esecuzione il `roscore` può avvenire sempre attraverso il meccanismo di scambio di messaggi via topic, ciò grazie a `rosserial`, un protocollo per incapsulare (_wrapping_) i messaggi standard di ROS, serializzarli e inviarli (attraverso un opportuno sistema di smistamento - _multiplexing_ - verso diversi topic/servizi) tramite seriale al microcontrollore.  
La libreria `rosserial` lavora secondo un meccanismo client/server per rendere possibile lo scambio di messaggi tra la scheda a microcontrollore e il computer di bordo: il PC che esegue ROS riveste il ruolo di server, mentre la scheda a microcontrollore agisce da client.  

![Rosserial protocol](img/rosserial.png)

Il server è realizzato attraverso un nodo ROS che fa uso di un'apposita libreria (si faccia riferimento al package ROS relativo al linguaggio di programmazione utilizzato: `rosserial_python`, `rosserial_server` per C++, `rosserial_java`) per implementare tali funzionalità; mentre la libreria client supporta tutte le piattaforme _Arduino_ o _mbed_ compatibili (si segnala fra le varie librerie client anche la presenza di `rosserial_stm32`).  
Il protocollo `rosserial` specifica la struttura a livello byte dei pacchetti dati che vengono scambiati dalle librerie client e server attraverso interfaccia di comunicazione seriale; ogni pacchetto contiene, oltre al dato uile, una serie di informazioni usate per la sincronizzazione e la validazione dei dati.  

![Rosserial packet](img/rosserial_packet.png)

Un pacchetto `rosserial` è composto da un _header_ che contiene informazioni generali sul dato e dei campi per la verifica dell'integrità del dato (_checksum_).  
- **Sync Flag**: questo byte è posto sempre a 0xFF, e sta a indicare l'inizio del pacchetto.
- **Sync Flag / Protocol version**: questo byte indica la versione del protocollo ROS, dove per Groovy è 0xFF, mentre per Hydro, Indigo, Jade, Kinetic, Melodic e Noetic è 0xFE.
- **Message Length**: questi due bytes contengono la lunghezza del dato che rappresenta il messaggio che si sta scambiando. Il _low byte_ viene inviato per primo, seguito dallo _high byte_ (notazione _little endian_).
- **Checksum over message length**: il valore del checksum viene utilizzato per verificare la validità del valore di lunghezza del dato ricevuto, calcolato secondo la formula 
`255 - ((message_length_low_byte + message_length_high_byte) % 256)`.
- **Topic ID**: il valore dell'ID consiste di due byte ed è usato come identificatore per distinguere il tipo di messaggio ricevuto. I valori di ID compresi tra 0 e 100 sono riservati per funzioni di sistema. I principali ID utilizzati dal sistema sono i seguenti:  
`uint16 ID_PUBLISHER=0`  
`uint16 ID_SUBSCRIBER=1`  
`uint16 ID_SERVICE_SERVER=2`  
`uint16 ID_SERVICE_CLIENT=4`  
`uint16 ID_PARAMETER_REQUEST=6`  
`uint16 ID_LOG=7`  
`uint16 ID_TIME=10`  
`uint16 ID_TX_STOP=11`
- **Serialized Message Data**: questo campo contiene il messaggio vero e roprio serializzato.
- **Checksum over topic ID and Message Data**: il valore di tale checksum è utilizzato per validare Topic ID e messaggio ricevuto, questo viene calcolato secondo la seguente formula: `255 - ((topic_ID_low_byte + topic_ID_high_byte + data_byte_values) % 256)`
- **Query Packet**: quando il server `rosserial` viene eseguito, questo richiede alcune informazioni al client, quale i nomi dei topic e i relativi tipi di messaggio. Quando viene effettuato lo scambio di tali informazioni, vengono utilizzati dei pacchetti speciali definiti _query packet_, il cui Topic ID e la dimensione del pacchetto sono settati ambedue a 0. Di seguito sono riportati i dati presenti in un `query packet`.  
`0xff 0xfe 0x00 0x00 0xff 0x00 0x00 0xff` 
Quando un client riceve un pacchetto di questo tipo, esso invia in risposta un messaggio al server contenente le seguenti informazioni.
`uint16 topic_id`  
`string topic_name`  
`string message_type`  
`string md5sum`  
`int32 buffer_size`  

> #### :warning: LIMITAZIONI DI ROSSERIAL
> Attraverso la libreria `rosserial` è possibile scambiare messaggi standard di ROS con dispositivi embedded basati su microcontrollore che dispongano di una periferica di comunicazione UART; tuttavia vi sono alcune limitazioni dovute all'hardware che potrebbero sollevare qualche problema. Per esempio, vincoli dovuti alle risorse di memoria (un numero di topic troppo elevato - dalla documentazione ufficiale: superiore ai 25 - potrebbe sollevare problemi) o tipi di dato non supportati, quali `Float64` e `String`.  

Sul computer di bordo vengono invece eseguiti gli altri package ROS, i quali sono orgamizzati come segue:
- `turtlebot3`: contiene i modelli del Turtlebot, i package contenti gli algoritmi necessari allo SLAM  e alla navigazione, il package per il controllo remoto e il package `bringup`;
- `turtlebot3_msgs`: contiene i file dei messaggi utilizzati negli altri package;
- `turtlebot3_simulations`: contiene i file necessari a simulare il Turtlebot su Gazebo.
- `turtlebot3_applications`: fornisce alcuni esempi di applicazioni con il Turtlebot 3.

## Installazione dei package del TurtleBot3
#### Installazione delle dipendenze
Prima di procedere all'installazione dei package relativi al Turtlebot 3, è consigliata l'installazione di alcuni package di dipendenza nella cartella principale di ROS: `/opt/ros/melodic`.
```bash
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

#### Installazione dei package del TurtleBot3
È possibile installare i package relatii al Turtlebot 3 all'interno del proprio workspace scaricandoli dal repository github ufficiale: https://github.com/ROBOTIS-GIT.
```bash
$ cd ~/ros_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ..
$ catkin_make
```
>##### :children_crossing: PROBLEMI DURANTE LA COMPLIAZIONE
> È possibile che si incontrino dei problemi in fase di compilazione con il pacchetto Python _em_; installare il pacchetto nel seguente modo:
> ```bash
> $ python -m pip install empy
> $ python3 -m pip install empy 
> ```

## Simulazione
Per lavorare nell'ambiente simulativo con Gazebo e Rviz è necessario specificare con quale modello di Turtlebot 3 si vuole operare (`burger`,`waffle` e `waffle_pi`). Ciò viene fatto specificando una variabile d'ambiente come riportato di seguito:
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
```
> ####  :fast_forward: SET ENVIRONMENT VARIABLE
> Per migliorare il flusso di lavoro, potrebbe essere conveniente includere il setaggio della variabile d'ambiente nel file `.bashrc` per evitare di dover eseguire il comando in ogni nuovo terminale.
> ```
> # Export Robot Model
> export TURTLEBOT3_MODEL=waffle_pi
> ```

### Spawn del modello in Gazebo
Per eseguire l'ambiente di simulazione Gazebo eseguire da terminale
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```
o 
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```
nel package `turtlebot3_description` sono presenti i file che descrivono la struttura del Turtlebot 3 attraverso file `*.urdf`, le specifiche fisiche per le simulazioni in Gazebo e i file `*.mesh` usati per modellare la struttura 3D del robot. Il package `turtlebot3_gazebo` contiene principalmente i file `*.launch` per effettuare lo _spawn_ del modello del robot in differenti scenari di Gazebo.

### Guida del Turtlebot 3
Per teleguidare il Turtlebot è disponibile il nodo `turtlebot3_teleop` nell'omonimo package, che permette la lettura dei dati da tastiera e l'invio di comandi di velocità sul topic `cmd_vel`. In un terminale digitare:
```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Visualizzazione su Rviz
Per visualizzare le percezioni del robot - come la sua posizione, il suo assetto, o i dati raccolti dai sensori - è possibile usare Rviz per raccogliere le informazioni pubblicate sui topic e visualizzarle graficamente:
```bash
$ roslaunch turtlebot3_gazebo turtlebort3_gazebo_rviz.launch
```
oppure far partire manualmente Rviz selezionando quali informazioni si vogliono visualizzare aggiungendo gli appositi oggetti.
```bash
$ rviz
```
Se viene seguita quast'ultima strada è necessario, oltre all'esecuzione _standalone_ di Rviz, l'esecuzione del nodo `robot_state_publisher` (dell'omonimo package)
```bash
$ rusrun robot_state_publisher robot_state_publisher
```
In funzione del modello di Turtlebot 3 scelto, sarà possibile aggiungere la visualizzazione di diversi sensori; per conoscere l'insieme di sensori con cui è equipaggiato il modello scelto, una possibile strada è quela di andare a visionare il contenuto del relativo file `*.gazebo.xacro` presente nel package `turtlebot3_description`.  
Per esempio, volendo utilizzare il Turtlebot 3 Waffle Pi, è possibile ottenere informzaioni sull'ambiente circostante attraverso i dati raccolti dal LiDaR e dalla Camera RGB. Dopo aver aggiunto il modello del robot, è possibile provare ad aggiungere oggetti di tipo `LaserScan` e `Camera`, dopodiché selezionare i topic `/scan` per il sensore laser e il topic `/camera/rgb/image_raw` per la camera.  

![TurtleBot 3 in Rviz](img/turtlebot_rviz.png)

## Costruzione di una Mappa
Un'aspetto molto importante al fine della navigazione autonoma di un robot è la capacità di costruire una mappa dell'ambiente in cui si muove. La capacià di costruzione autonoma permette di non dover effettuare in precedenza un'operazione di costruzione, in alcuni casi non possibile.  
In ROS le mappe di navigazione possono essere rappresentate con una griglia bidimensionale (nel caso di movimenti su un piano), dove ogni casella della matrice contiene un valore numerico che corrisponde al _"grado di occupazione"_ della porzione di spazio corrispondente: il bianco identifica uno spazio libero, il nero spazio occupato, il grigio è associato a spazio ignoto (non ancora splorato). I file rappresentanti una mappa possono essere salvati come file immagine in uno dei classici formati: `*.png`, `*.jpg`, `*.pmg`. Associato ad ogni immagine vi è un file `*.yaml` contenente informazioni aggiuntive su come interpretare la mappa: la risoluzione (la dimensione del lato di una cella espressa in metri), l'origine della mappa, e i valori di soglia (_threshold_) necessari a stabilre se una cela può essere considerata libero, occupata o ignota.  
ROS mette a disposizione più di un package per la costruzione delle mappe. Tra i vari package, uno dei più noti è `gmapping`, che registra i valori acquisiti dai sensori laser mentre simultaneamente localizza il robot facendo uso dei dati odometrici pubblicati. Affinché la costruzione della mappa avvenga in maniera efficace, è necessario ripercorrere più volte l'intera mappa, prestando attenzione ad eseguire movimenti lenti (soprattuto nelle operazioni di rotazione) affinché l'algoritmo riesca a raccogliere quanti più dati possibile dell'ambiente circostante.  
Utilizzando il nodo `slam_gmapping` del package `gmapping` è possibile cosruire una mappa dell'ambiente di simulazione entro cui ci si sta muovendo.

![Mapping process on the TurtleBot 3](img/mapping_process.png)

Prima di lanciare il nodo `slam_gmapping` è necessario impostare le dimensioni della mappa che si vuole costruire, in modo da poter dimensionare opportunamente la matrice che la dovrà alloggiare. Ciò viene fatto andando a configurare nel parameter server i seguenti parametri:
```bash
$ rosparam set /slam_gmapping/xmax 10  
$ rosparam set /slam_gmapping/xmin -10
$ rosparam set /slam_gmapping/ymax 10  
$ rosparam set /slam_gmapping/ymin -10
```
Giunti a tale punto è possibile eseguire
```bash
$ rosrun gmapping slam_gmapping
```
Su Rviz è possibile provare ad aggiungere l'oggetto _Map_ e settare il topic `/map`, per visualizzare la mappa via via che la si costruisce. Assicurarsi che il _fixed frame_ di Rviz sia impostato sul frame `map` per la corretta visualizzazione del movimento.  
È possibile provare a muovere il Turtlebot all'interno della mappa utilizzando il nodo di teleoperazione, mentre si osserva il processo di creazione della mappa in real-time su Rviz.

![Building a map](img/turtlebot_map.png)

Quando si è raggiunto un livello di costruzione della mappa considerato accettabile (tutti gli spazi sono stati visitati e non vi sono più celle ignote all'nterno dello spazio desiderato), è possibile usare il nodo `map_saver` dell'omonimo package per salvare sul disco sia la mappa che il suo file di configurazione. In un nuovo terminale eseguire:
```bash
$ rosrun map_server map_saver
```
e, a operazione terminata, verificare la presenza dei file nella cartella `/home` dell'utente. Il nodo `map_saver` genera un'immagine che contiene la griglia di occupazione e il file `*.yaml` contenente le informazioni aggiuntive per interpretare la stessa.
```
image: map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
È possibile spostare ambedue i file nella cartella `map/` nel package per usarla durante le applicazioni che richiedono la navigazione autonoma nell'ambiente mappato.   
È possibile usare il package `gmapping` anche con il robot personalizzato costruito nelle lezioni precedenti, per ricostruire la mappa dell'ambiente personalizzato che si era creato.

![Building a map with own robot](img/smr_map.png)
