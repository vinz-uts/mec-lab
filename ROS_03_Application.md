# Autonomous Application
With the capabilities of localization and navigation, the vehicle can be considered autonomous, so it is possible to write some applications that use these skills to perform some tasks independently.
Let's to create a new package ```turtlebot3_app``` to create some application for the Turtlebot
```bash
$ catkin_create_pkg turtlebot3_app rospy roscpp std_msgs geometry_msgs nav_msgs
```

## Patrolling
One of the most common application for a mobile robot is to patrol around the world, for surveillance purpose or for search some object, or for monitoring the environment collecting interesting data.  
Patrolling can be performed using different strategy and algorithm: travel through _waypoints_ or doing _random walk_.  
In the next example are developed a simple scheme to perform patrolling via waypoints. In the ```scripts/``` folder of the package create a new file ```patrol.py```
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
        # [(x,y,ϑ),(x,y,ϑ),(x,y,ϑ)]
        self.waypoints = [(1.6,-0.6,0.7199),(1.5,1.5,2.7586),(-1.5,1.5,-2.0984),(-1.5,-1.5,-0.4026)]
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

Because the ```tf``` python library dosen't work with python 3, the above script is write using the python 2 interpreter (this is a clear example of ROS interoperability: the programmer can use different language for write different node in own applications).  
Import the action messages for use the ```move_base``` node
```python
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
```
Define a ```Patrol``` class, initializing the node and specifying a list of waypoints, where each of them is a tuple that contains the cartesian position (x,y) and the desidered orientation ![\vartheta](https://render.githubusercontent.com/render/math?math=%5Cvartheta). Next create a ```SimpleActionClient``` for use ```move_base``` actions and wait that the server is running.

```python
def __init__(self):
    rospy.init_node('patrol')
    # ------ PARAMETERS ------
    # list of waypoint:
    # [(x,y,ϑ),(x,y,ϑ),(x,y,ϑ)]
    self.waypoints = [(1.6,-0.6,0.7199),(1.5,1.5,2.7586),(-1.5,1.5,-2.0984),(-1.5,-1.5,-0.4026)]
    # ------ ACTIONS ------
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
```
> #### :pushpin: CHOISE OF WAYPOINTS
> For choose the waypoints and know desidered coordinates and the orientation, you can help yourself using Rviz to select the desidered goal using the "2D Nav Goal" button and see the ```MoveBaseActionGoal``` published on the ```/move_base/goal``` topic using
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
> In the ```goal.target_pose.pose``` field see the ```orientation``` for (x,y) coordinates, and the ```orientation``` for the desidered heading. The orientation is expressed by a _quaternion_ to convert it in _Euler angles_, open a python interpreter and use the ```tf.transformations.euler_from_quaternion``` function
> ```bash
> $ python
> Python 2.7.17 (default, Apr 15 2020, 17:20:14) 
> [GCC 7.5.0] on linux2
> Type "help", "copyright", "credits" or "license" for more information.
> >>> import tf
> >>> q = tf.transformations.euler_from_quaternion([0,0,0.35,0.93])
> >>> q
> (0.0, -0.0, 0.719897052054812)

In the ```loop``` function cycles over the waypoints, sending at each time a ```MoveBaseGoal``` for the corresponding waypoint doing the orientation transformation between _Euler's angles_ and _quaternion_ using the ```tf``` functions. For the full documentation about the ```MoveBaseGoal``` fields browse http://docs.ros.org/fuerte/api/move_base_msgs/html/msg/MoveBaseActionGoal.html.  
Run the Turtlebot in autonoumus configuration  
```
$ roslaunch tutlebot3_navigation turtlebot3_localization.launch
```
then, in a new terminal
```
$ roslaunch tutlebot3_navigation move_base.launch
```
To run the ```patrol``` node type
```bash
$ chmod +x ~/ros_ws_src/turtlebot3_app/scripts/patrol.py
$ rosrun turtlebot3_app patrol.py
```
Try to add _Odometry_ object in Rviz to visualize the path of the robot during the patrolling.

![Patrolling](img/patrolling.png)

## More _intelligent_ robot
To perform more sophisticated task, is useful to describe and model the robot behaviors that follow one another during the task. To do this it is a good idea to take advantage of one of the fundamentals of the computer science: the **state machine**.  
The main idea is that the robot can be in one of a finite number of **states**, each of which maps to a behavior (e.g. _moving_, _waiting_, *collecting_data*, etc). When one state ends, the system immediately moves into another state (for example: changing from _waiting_ to _moving_ when the robot start to navigate around the world).  
Unlike the common interpretation used in state machines, states of the system do not represent a _static_ configuration of the robot, but represent the _dynamic_ state that implies the execution of a task.  
The **transitions** between the states specify the structure of the state machine, and _labels_ are attached to each of them for identify the conditions under which they are followed (e.g. _success_, _aborted_, _failed_).  

![Example of state machine](img/state_machine_01.png)

### SMACH
State machines in ROS can be built using the ```smach``` framework and its ROS-specific extensions in ```smach_ros```. The core of ```smach``` are ROS-indipendent and contains a few libraries wrote in Python. This framework, according to a _composite software design pattern_, export two main interfaces (_State_, _Container_), which allow to define hierarchical state machines.
#### State
The _State_ objects represent the "states of execution" for the robot and each of them specifis all the possible _outcomes_ of the state. A ```State``` object must implements the ```execute``` function that performs the behavior associated with the state and return the _outcome_ that identify how is terminated the task.
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
A _Container_ is a collection of one or more states that are executed with a defined policy. The most simple _Container_ is ```StateMachine``` that allows to model the state machine like a flowchart, where the ```State``` are interconected in function of the _outcomes_ generated. The ```StateMachine``` object is also a _States_ because it implements its interfaces, so it have to define its _outcomes_.

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

Another very useful _Container_ is ```Concurrence```, whose policy calls for the simultaneous execution of all its states. When ```Concurrence``` container is initializing, is possibile to define a python dictionary (```outcome_map```) that specifies the outcomes of the container as a combination of the child outcomes. Is also possible to implement two _callback_: ```child_termination_cb``` and ```outcome_cb```, to define what is the mechanism of the container when each child state terminate or when the last child finish.  
SMACH's _Conteiners_ allow to define a variables dictionary inside (```userdata```), that contains variables shared by its states.  
Follow an example of a simple state machine implemented using ```smach```.

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
Try to run the example, typing
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
Using ```smach``` is possible to perform the patrolling behavior using a state machine. This because patrolling can be see as a flowchart where the robot drive to a waypoint to anhoter continuously. For perform the task, we only need to implement a single state corresponding to driving to a particular waypoint and define the order on which reach the target pose:
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

The ```Waypoint``` class represent the _state_ of "_driving to the waypoint w_". In the ```__init__``` method, that recives the waypoint as argument, is created the target point as ```MoveBaseGoal```, and the ```MoveBaseAction``` _client_ are initialized. In the ```execute``` method the ```goal``` is sent to the ```move_base``` server and if the goal is reached the _outcome_ is setted to ```success```.  
The ```PatrolAutoma``` class realizes the ROS node and build the state machine specifying a _State_ for each waypoints, where the next state is obtained as an increment with rollover on the waypoints list.
Start the turtlebot 
```
$ roslaunch tutlebot3_navigation turtlebot3_localization.launch
```
then
```
$ roslaunch tutlebot3_navigation move_base.launch
```
and execute the ```patrol``` node
```bash
$ chmod +x ~/ros_ws_src/turtlebot3_app/scripts/patrol_automa.py
$ rosrun turtlebot3_app patrol_automa.py
```

Another way to implement the patrol as a state machine, is using the ```SimpleActionState``` class from the ```smach_ros``` package, that realizes a _State_ for use an ```actionlib``` action. For ```SimpleActionState``` states, its outcomes are automatic generated from the action result.
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
