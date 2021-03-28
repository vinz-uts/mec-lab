# Classi e Oggetti
Python supporta, oltre alla programmazione funzionale, anche il paradigma di **programmazione orientata agli oggetti**. Lungi dal voler dare una trattazione esaustiva di tale argomento, si procede ad illustrare i concetti principali che carattarezziano tale approccio di progettazione e implementazione software.  
Elemento principale sono le **classi**: una classe definisce una struttura base che modella una qualche entità astratta. In particolare, una classe ha lo scopo di forire una modellizzazione di un entità astratta fornendone una descrizione in termini di:
- **attributi**: rappresentati da variabili e parametri
- **comportamenti**: modellati attraverso funzioni, dette _metodi_.

La classe costituisce uno "stampo" per creare **oggetti** del tipo definito dalla classe, ogni oggeto creato a partire dalla classe è detto _istanza_ della classe.  
Una classe è definita attrverso la parola chiave `class` e implementa una serie di funzioni proprie (blocco di codice innestato) che rappresentano i comportamenti esportati dall'entità che questa rappresenta. Tra tali metodi ve ne sono alcuni speciali, tra cui il metodo `__init__` che è il metodo che viene chiamato (implicitamente) quando si vuole creare una nuova istanza della classe; tale metodo è detto anche _costruttore_.  
> :mag: La realtà dei fatti è un po' diversa e si invita chi è già esperto del paradigma di programmazione ad oggetti ad approfondire la questione: il metodo `__init__` non equivale ai costruttori presenti negli altri linguaggi, poiché non crea l’istanza, ma la inizializza solamente. Dietro le quinte viene invocato dapprima il metodo `__new__` che crea un'istanza della classe. 

```python
class ClassName:
    def __init__(self, params):
        '''Constructor'''
        self.params = params
        
    def method(self, a, b):
        '''Method'''
        operations on self.params, a, b
```

Tutti i metodi di una classe devono definire un parametro aggiuntivo chiamato `self` (un po' come il famoso `this` di Java) che rappresenta l'istanza dell'oggetto che invoca il metodo. Per intenderci meglio: quando si invoca il metodo il parametro `self` non deve essere passato esplicitamente, ma è automaticamente sostituito dall'istanza su cui è stato invocato il metodo secondo la notazione `instance.method(params)`. L'espressione precedente è puro _zucchero sintattico_ in quanto dietro le quinte Python effettua una chiamata del tipo `ClassName.method(instance)` perciò i due modi producono lo stesso risultato.

> :pencil: È possibile fare chiarezza sui concetti di classe e istanze effettuando qualche prova attraverso l'interprete interattivo.
> ```python
> >>> class Test:
> ...     def __init__(self):
> ...         pass
> ...     def method(self):
> ...         print('self is:', self)
> ... 
> >>> t1 = Test()
> >>> t1
> <__main__.Test object at 0x7f39f5689cd0>
> >>> t2 = Test()
> >>> t2
> <__main__.Test object at 0x7f39f567d550>
> >>> t1.method()
> self is: <__main__.Test object at 0x7f39f5689cd0>
> >>> t2.method()
> self is: <__main__.Test object at 0x7f39f567d550>
> >>> Test.method(t1)
> self is: <__main__.Test object at 0x7f39f5689cd0>
> >>> Test.method(t2)
> self is: <__main__.Test object at 0x7f39f567d550>

Gli attributi di una classe sono espressi da variabili che identificano delle proprietà che ogni istanza deve possedere, come un nome, un numero di serie, ecc. Ogi istanza potrà avere dunque un valore diverso associato ai propri attributi, questi sono definiti all'interno del metodo `__init__`, nel qual caso sono detti _attributi d'istanza_; qualora invece ci debbano essere dei parametri condivisi da tutte le istanze di una stessa classe, si definiscono appena dentro la classe quelli che sono chiamati _attributi di classe_.

> :pencil: Un classico esempio per entrae in confidenza con classi, istanze e metodi è fornito dalla seguente classe:
> ```python
> class Person:
>     planet = 'Earth'                            <-- class atribute
>     def __init__(self, name, dd, mm, yy):
>         self.name = name                        <-- instance atribute
>         self.date = (dd,mm,yy)                  <-- instance atribute
>   
>     def get_age(self, curr_year):
>         age = curr_year-self.date[-1]
>         if age < 0:
>             print('Year must be bigger then '+str(self.date[-1]))
>             return 0
>         return age
> ```
> Di seguito è riportato il codice relativo alla creazione di due istanze della stessa classe, rappresentanti due oggetti - entità - diverse, ognuna con i propri tratti caratteristici.
> ```python
> p1 = Person('Franco',1,10,1970)
> p2 = Person('Gigi',29,2,1984)
> print('Age of '+p1.name+', living on '+p1.planet+': '+str(p1.get_age(2021)))
> print('Age of '+p1.name+', living on '+p1.planet+': '+str(p2.get_age(2021)))
> ```
>
> Eseguire il file `person.py` da terminale per mostrare i risultati delle operazioni
> ```bash
> $ python3 person.py
> ```

Le classi supportano anche diversi metodi "speciali" che sono identificati dalla presenza di due underscore prima e dopo il nome. Questi metodi, in genere, non vengono chiamati direttamente con la sintassi `instance.__method__`, ma vengono invocati implicitamente e automaticamente in situazioni particolari; un esempio, già incontrato, è dato dal metodo `__init__` che viene invocato automaticamente quando si fa ricorso alla sintassi `ClassName(param)` per creare un nuovo oggetto di quel tipo. Altri metodi speciali sono quelli che vengono invocati dietro le quinte quando si utilizzano gli operatori relazionali, aritmetici, ecc, fra oggetti di una stessa classe, oppure metodi come `__str__`, `__len__`, `__hash__`.  
| Operatore | Metodo   | Descrizione |
| --------- | ------   | ----------- |
| `==`      | `__eq__` | Definisce il criterio di uguaglianza fra oggetti diverso da quello standard basato sugli indirizzi di memoria.|
| `>` `>=`  | `__gt__` `__ge__`| Definisce il criterio di maggioranza tra oggetti. |
| `<` `<=`  | `__lt__` `__le__` | Definisce il criterio di minoranza tra oggetti. |
| `+` | `__add__` `__radd__` `__iadd__`| Definisce l'operazione di somma tra oggetti. |
| `-` | `__sub__` `__rsub__` `__isub__`| Definisce l'operazione di sottrazione tra oggetti. |
| `*` | `__mul__` `__rmul__` `__imul__`| Definisce l'operazione di prodotto tra oggetti. |
| `/` | `__truediv__` `__rtruediv__` `__itruediv__`| Definisce l'operazione di divisione tra oggetti. |

Come si può notare i metodi speciali assocciati ad ogni operatore aritmetico (vale anche quelli non riportati o quelli binari: `//`, `%`, `>>`, `<<`, `&`, `|`, `^` ) sono tre. Il metodo principale associato all'operatore (supponiamo `+`) è `__add__`, questo è usato dal linguaggio per risolvere espressioni del tipo `myobj + obj`, in tal caso l'interprete effettua una chiamata del tipo `myobj.__add__(obj)`. Ma cosa succede se l'istruzione è scritta al contrario, ovvero `obj + myobj`? Se l'oggetto `obj` è un'altra istanza della stessa classe di `myobj` allora poco cambia, in quanto l'invocazione `obj.__add__(myobj)` darebbe lo stesso risultato del caso precedente; se invece `obj` è di un tipo diverso da `myobj` che magari non implementa il metodo `__add__`, o non ha conoscenza del tipo di `myobj`, allora viene restituito il valore speciale `NotImplemented` che l'interprete non è in grado di gestire, sollevando un errore di tipo `TypeError` che può interreompere l'esecuzione del programma. Affinché la proprietà commutativa degli operatori sia posibile è possibile implementare il metodo `__raddr__` il quale viene invocato proprio nei casi in cui l'operazione `obj.__add__(myobj)` non vada a buon fine a causa di un `NotImplemented`, in questa situazione l'interprete Python provvede a verificare se l'operando a destra implementa tale metodo e, in caso affermativo, provvede ad eseguire l'istruzione `myobj.__raddr__(obj)`. I metodi speciali con una "i" davanti al nome servono invece a implementare le operazioni di assegnamento immediato, ad esempio `__iadd__` è invocato quando viene usato l'operatore `+=`.

> :pencil: È possibile aggiungere alla classe preceente dei metodi _naive_ per stabilire quale persona abbia l'eta maggiore usando gli operatori relazionali.
> ```python
> class Person:
>     planet = 'Earth'
>     def __init__(self, name, dd, mm, yy):
>         self.name = name
>         self.date = (dd,mm,yy)
>   
>     def get_age(self, curr_year):
>         age = curr_year-self.date[-1]
>         if age < 0:
>             print('Year must be bigger then '+str(self.date[-1]))
>             return 0
>         return age
>     
>     # used when call '>'
>     def __gt__(self, p):
>         if self.date[2] < p.date[2]:
>             return True
>         elif self.date[2] > p.date[2]:
>             return False
>     
>      # used when call '<'
>     def __lt__(self, p):
>         return self.date[2] > p.date[2]
> 
> p1 = Person('Franco',1,10,1970)
> p2 = Person('Gigi',29,2,1984)
> 
> if p1 > p2:
>     print(p1.name+' is older than '+p2.name)
> elif p1 < p2:
>     print(p1.name+' is younger than  '+p2.name)
> else:
>     print(p1.name+' and '+p2.name+' have the same age')
> ```

## Ereditarietà
Python supporta il meccanismo dell'ereditarietà che permettere di _estendere_ (**extends**) una classe già esistene - più generale - con una classe - più specializzata - che ne eredita tutti gli attributi e metodi e ne esporta di nuovi più specifici. La creazione di una classe erede prevede di specificare in fase di dichiarazione la classe padre che vuole estendere, ciò viene fatto con la seguente sintassi: `class ChildClass(SuperClass):`. Python supporta anche il meccanismo più complesso dell'**ereditarietà multipla**, consentendo di specificare tutte le _super class_ da cui eredità, separate da virgole. È anche possibile, nella sottoclasse, sovrascrivere (_override_) e quindi ridefinire metodi definiti dalla superclasse.  
All'interno della sottoclasse, se si vuole far riferimento ad un metodo della superclasse è possibile usare la funzione built-in `super` (la quale restituisce un riferimento alla classe base) per accedere al metodo corrispondente definito nella classe base.

> :pencil: Una classe erede _specializza_ una classe precedentemente creata, rappresentando una entità più specifica della precedente. Per ereditare da una classe definita in un altro file (ad esempio `person.py`) è necessario ricorrere all'operazione di `import`
> ```python
> from person import Person
> class Student(Person):
>     def __init__(self, name, dd, mm, yy, id_num):
>         super().__init__(name,dd,mm,yy) # call super class constructor
>         self.id_num = id_num
>   	
>     # Override methods
>     # used when call '>'
>     def __gt__(self, p):
>         return self.id_num < p.id_num
>       
>     # Override methods
>     # used when call '<'
>     def __lt__(self, p):
>         return self.id_num > p.id_num
> ```
> Nell'esempio La classe `Student` eredita dalla classe `Person`, in quanto uno studente è sì una persona generica, ma con degli attributi in più come un numero di matricola. In questo caso è probabilmente più interessante sapere quale studente si sia immatricolato prima di un altro per avere un criterio di ordinamento, anziché basarsi sull'età anagrafica. È da notare che uno studente è una persona, ma una persona non necesariamente deve essere uno studente: l'ereditarietà è una relazione unidirezionale.

> ## :pencil: Controllori PID
> Nella pratica industriale e generalmente del controllo automatico, si utilizzano molto spesso dei regolatori "standard", grazie ai quali è possibile ottenere le prestazioni desiderate tramite la taratura (_tuning_) di pochi parametri. I controllori **PID** rientrano in questa famiglia, e possono essere implementati in forma digitale in diversi modi. Di seguito si farà uso del paradigma ad oggetti per creare delle entità del tutto generiche che potranno essere inizializzate caso per caso, in funzione del problema di controllo che si deve risolvere.  
Il **PID** è un controllore dinamico che fornisce un'azione di controllo proporzionale all'errore `e(t)`, al suo integrale ed alla sua derivata. Matematicamente si può definire attraverso la seguente equazione:  
> <center> <a href="https://www.codecogs.com/eqnedit.php?latex=u(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int_0^t&space;e(\tau)d\tau&space;&plus;&space;K_d&space;\frac{d}{dt}e(t)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int_0^t&space;e(\tau)d\tau&space;&plus;&space;K_d&space;\frac{d}{dt}e(t)" title="u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{d}{dt}e(t)" /></a> </center>
> Affinché sia possibile implementarne una versione digitale è necessario trasformare l'equazione precedente nella sua forma discretizzata:
> <center> <a href="https://www.codecogs.com/eqnedit.php?latex=u_k&space;=&space;K_p&space;e_k&space;&plus;&space;K_i&space;\sum_{i=0}^k&space;e_i&space;&plus;&space;K_d&space;\frac{e_k-e_{k-1}}{T_c}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u_k&space;=&space;K_p&space;e_k&space;&plus;&space;K_i&space;\sum_{i=0}^k&space;e_i&space;&plus;&space;K_d&space;\frac{e_k-e_{k-1}}{T_c}" title="u_k = K_p e_k + K_i \sum_{i=0}^k e_i + K_d \frac{e_k-e_{k-1}}{T_c}" /></a> </center>
> 
> Sebbene l'implementazione di questa formula sia possibile senza troppo dispendio di energie dal punto di vista della programmazione, dal punto di vista computazionale risulta poco performante in quanto il calcolo dell'integrale mediante sommatoria richiederebbe la memorizzazione di tutti i campioni passati e, ad ogni istante, un ciclo di somme il cui numero di addendi cresce di continuo. Una formulazione ricorsiva più efficiente è la seguente:
> <center> <a href="https://www.codecogs.com/eqnedit.php?latex=u_k&space;=&space;K_p&space;e_k&space;&plus;&space;K_i&space;\underbrace{(S_{k-1}&space;&plus;&space;T_c&space;e_k)}_{S_k}&space;&plus;&space;K_d&space;\frac{e_k-e_{k-1}}{T_c}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u_k&space;=&space;K_p&space;e_k&space;&plus;&space;K_i&space;\underbrace{(S_{k-1}&space;&plus;&space;T_c&space;e_k)}_{S_k}&space;&plus;&space;K_d&space;\frac{e_k-e_{k-1}}{T_c}" title="u_k = K_p e_k + K_i \underbrace{(S_{k-1} + T_c e_k)}_{S_k} + K_d \frac{e_k-e_{k-1}}{T_c}" /></a> </center>
> 
> Dove il termine `Sk` rappresenta un accumulatore dell'errore passo dopo passo e sostituisce la sommatoria.
> > :mag: I controllisti già sanno che la derivazione della formula discreta del PID dipende dal tipo di approssimazione utilizzata (_differenze in avanti_, _differenze all'indietro_, _metodo di Tustin_), come anche conoscono l'esistenza della formulazione del PID con l'annesso filtro passa-basso o i criteri di taraura dei parametri (la taratura "per tentativi" è un'invenzione degli ultimi tempi, da non considerarsi autorevole, un ingegnere del controllo dovrebbe dunque ricorrere a tecniche più scientifiche. Attenzione che le tecniche _empiriche_ non sono equivalenti alla scelta aleatoria dei parametri).  
>
> L'entità PID può essere modellata atrraverso una classe. Dalla precedente formula ricorsiva è possibile inferire sia l'algoritmo di controllo da implementare nell'apposito metodo, sia gli attributi (parametri e variabili) di cui ogni oggetto PID deve disporre.
> ```python
> class PID:
>     def __init__(self, Kp, Ki=0, Kd=0, Tc=0.1):
>         self.Kp = Kp
>         self.Ki = Ki
>         self.Kd = Kd
>         self.Tc = Tc
>         self.Sk = 0
>         self.ek_1 = 0
>         
>     def calculate(self, yk, r):
>         ek = r-yk
>         self.Sk += ek*self.Tc
>         uk = self.Kp*ek + self.Ki*self.Sk + self.Kd*(e - self.ek_1)/self.Tc
>         self.ek_1 = ek
>         return uk
>       
>     def reset(self):
>         self.Sk = 0
>         self.ek_1 = 0
> ```
> Quando si lavora con attuatori reali, questi presentano dei limiti fisici di funzionamento in termini del segnale di controllo (forza, coppia, tenisione) che riescono a generare. Tali valori rappresentano il limite di **saturazione** dell'attuatore, ed è dunque opportuno fare in modo che l'algoritmo di controllo non generi azioni di controllo superiori a questi valori: sia perché irrealizzabili, sia perchè l'avorare ai limiti porta a una maggiore usura degli attuatori. In caso di saturazione, è opportuno inoltre gestire opportunamente l'effetto integrale il quale altrimenti potrebbe generare fenomeni di _windup_. Gli algoritmi di **anti-windup** prevedono di bloccare o decrementare l'accumulo dell'integrale qualora il sistema vada in saturazione, per evitare un rallentamento della reattività del controllo. È possibile definire una nuova classe che aggiunga tali funzionalità alla classe PID precedente, in accordo con il meccanismo dell'ereditarietà. 
> ```python
> class PID_Saturation(PID):
>     def __init__(self, Kp, Ki=0, Kd=0, Tc=0.1 u_max=1):
>         super().__init__(Kp,Ki,Kd,Tc) # call super class constructor
>         self.u_max = abs(u_max)
>         
>     # Override methods
>     def calculate(self, yk, r):
>         uk = super().calculate(yk,r)
>         if not(-self.u_max <= uk <= self.u_max):
>             self.ek_1 -= (r-yk)*self.Tc # anti-windup
>             return (uk/abs(uk))*self.u_max
>         return uk
> ```

