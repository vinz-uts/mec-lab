# ROS Getting Started

## ROS Installation
Brief guide to install [ROS Melodic Morenia](http://wiki.ros.org/melodic) on the Linux Operating System distibution [Ubuntu 18.04.4 LTS Bionic Beaver](https://www.ubuntu-it.org/download). For more dettails, please consult the [official web guide](http://wiki.ros.org/melodic/Installation/Ubuntu).

### Setup sources.list
Add official ROS packages repository to Linux packages repositories to support ```apt``` installation.
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Set up keys
 Add a public key in order to download the package from the ROS repository.
```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Installation
Update packages at the last version, next download and install the most complete installation of ROS Melodic that include robot-generic libraries, tools for 2D/3D simulation, and more over useful packages.
```
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
```

### Initialize rosdep
Enable dependencies system for source compiling and is required to run some core components in ROS.
```
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep update
```

### Dependencies for building packages
Install several ROS packages with useful tools that is frequently used in ROS applications.
```
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Environment setup
Add and save on ```~/.bashrc``` file the environment settings to found ros command and main packages when starts each terminal. 
```
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
Also, directly modify the hidden file ```~/.bashrc```, adding at the end of file the equivalent following lines:
```bash
# Set ROS Melodic environment  
source /opt/ros/melodic/setup.bash
```

## Workspace Creation
Crate a workspace folder for a ROS project. It's good practice create a new workspace for each new project you want to start.
```
$ mkdir -p ros_ws/src
$ cd ros_ws/
```
set _python 3_ as default (needed only the first time) for the **catkin** build system, and run ```catkin_make```.

```
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
$ catkin_make
```
Once building process is finished ```CMakeList.txt``` file and ```build``` and ```devel``` folder have been created. The build system basically uses _CMake_ (_Cross Platform Make_), and the build environment is described in the ```CMakeList.txt``` file. The build related files are saved in the ```build``` folder, and the execution related files are saved in the ```devel``` folder.  
Import the setting file associated to the project workspace
```
$ source ~/ros_ws/devel/setup.bash
```
or modify _~/.bashrc_ file adding at the end the lines
```bash
# Set ROS Melodic environment  
source /opt/ros/melodic/setup.bash  
# Set ros_ws workspace environment  
source ~/ros_ws/devel/setup.bash
```

### Create a package
Project can be organized in different packages for improve readability, navigability and reusability of the code. Packages must be created in the ```src``` folder of the workspace
```
$ cd ~/ros_ws/src
$ catkin_create_pkg test_pkg
```
Using the ```catkin_create_pkg``` command two files have been created: the manifest ```package.xml```, containing package information such as name, author, license and dependent packages; the ```CMakeList.txt``` file for build configuration.  
> :warning: **WARNING**  
> Nested packages is not allowed in ROS, for group multiple packages as a simple logical package use ROS **metapackage**. It requires simple ```CMakeList.txt``` and ```package.xml``` configuration:  
> ```xml 
> <?xml version="1.0"?>
> <package format="2">
>   <name>METAPACKAGE_NAME</name>
>   <version>0.0.1</version>
>   <description>METAPACKAGE_DESCRIPTION</description>
>   <buildtool_depend>catkin</buildtool_depend>
>   <export>
>     <metapackage />
>   </export>
> </package>
> ```
> ```cmake
> cmake_minimum_required(VERSION 2.8.3)
> project(PACKAGE_NAME)
> find_package(catkin REQUIRED)
> catkin_metapackage()
> ```
Also two default folder have been created in the package directory: ```src```, for source code folder and ```include```, for header file folder. Package can contains also other folder depending on his purpose, commonly:
> ```/action``` user's custom _actions_  
> ```/include``` C++ header files  
> ```/launch``` launch files  
> ```/msg``` user's custom _messages_  
> ```/scripts``` or ```/nodes``` python scripts  
> ```/src``` C++ source files  
> ```/srv``` user's custom _services_  

Next the name of package, user can be add dependent package, for example to add ```std_msgs``` (that contains ROS standard message) and ```rospy``` (ROS client library for Python language) the creation package command becomes:
```
$ catkin_create_pkg test_pkg std_msgs rospy
```
that automatically add the dependencies to package.xml and CMakeList.txt files
```xml 
<?xml version="1.0"?>
<package format="2">
  <name>test_pkg</name>
  <version>0.0.0</version>
  <description>test_pkg package for examples code</description>
  <maintainer email="user_name@todo.todo">user_name</maintainer>
  <license>TODO</license>
  
  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  
  <export> </export>
</package>
```
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```

## Node and Communications
ROS is based on a graph of indipendent units that communicate through _messages_. A **node** refers to the smallest unit of processor running in ROS (think of it as one executable program). ROS recommends to creating one single node for each purpose, for improve reusability of the code. Communications between nodes can happen mainly with three methods:  
- Topics
- Services
- Actions


### Topics
The simplest way for send or receive data between nodes via messages is using the abstract mechanism of **topic**, similar to the _software design pattern Observer_.  
This communication mechanism requires the presence of a **publisher node** that writes messages (which are variables such as integer, string, etc, or more structured data) on the topic and one or more **subscribers nodes** that recive this messages.  
Let's try to implement a first type of communication between nodes using topic abstractions.
#### Publisher Node
The intent is to create a node that publishes the value of a counter each second on a defined topic. Below a simple Python script to do this:
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node('counter')
pub = rospy.Publisher('count_topic', Int32)
rate = rospy.Rate(1)
count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
```
First two lines
```python
import rospy  
from std_msgs.msg import Int32
```
import the _ROS client library_ for Python language and the _ROS standard message library_ that will use to send 32-bit integer values on the topic.

```python
rospy.init_node('counter')
```
initialize a **ROS node** called ```'counter'```.

```python
pub = rospy.Publisher('count_topic', Int32)
```
create an instance of a _Publisher_ object for send 
```Int32``` messages on ```'count_topic'``` topic.

```python
rate = rospy.Rate(1)
count = 0
```
create an object for setting the rate (in Hertz) which will used to publish on the topic and inizialize the ```conut``` variable to 0.  
The loop
```python
while not rospy.is_shutdown():  
    pub.publish(count)  
    count += 1  
    rate.sleep()
```
iterates while the node is not shutted down (in many cases is similar to a ```while True:``` command).
The life-cycle of the node is simple: publish the value of the count on the topic, update the ```count``` vairable, and sleep for a time to make sure that the body of the ```while``` loop runs at approximately 1 Hz.  
To run the node save the file as ```counter_node.py``` in the ```scripts``` folder of the ```test_pkg``` package, next ad execute permission to python script.
```
$ cd ~/ros_ws/src/test_pkg/scripts
$ chmod u+x counter_node.py
```
Open a teminal and run
```
$ roscore
```
then run the publisher node in a new terminal:
```
$ rosrun test_pkg counter_node.py
```
In other terminal it's possible to check the active node with the command
```
$ rosnode list
/counter
/rosout
```
or the list of the active topic using
```
$ rostopic list
/count_topic
/rosout
/rosout_agg
```
For display the messages published by the ```counter``` node run
```
$ rostopic echo /count_topic 
data: 261
---
data: 262
---
data: 263
---
data: 264
---
data: 265
---
```
#### Subscriber Node
Suppose to modify the previous _publisher node_ to send random data like a sensor (```writer_node.py```). A _subsciber node_ can read the published values and store them in a buffer.  
When a node is subscribed to a topic, it has to define a ***callback function***, that will invoked whenever any publisher will write a message.
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

global buffer, buffer_size
buffer_size = 5
buffer = []

def read_callback(msg):
    global buffer, buffer_size
    if len(buffer) == buffer_size:
        buffer.pop(0)
    buffer.append(msg.data)
    print(buffer)

rospy.init_node('reader')
sub = rospy.Subscriber('data_topic', Int32, read_callback)
rospy.spin()
```
Because the buffer is used whenever a _callback_ is called it has be defined as a _global_ variable for store his values in the time
```python
global buffer, buffer_size  
buffer_size = 5  
buffer = []
```
In this example the buffer is initialized empty with a maximum size of 5.  
The ```read_callback``` will invoked each time a publisher send a message ```msg``` on the topic:
```python
def read_callback(msg):  
    global buffer, buffer_size  
    if len(buffer) == buffer_size:  
        buffer.pop(0) 
    buffer.append(msg.data)  
    print(buffer)
```
At the  beginning of the function are specified that ```buffer``` and ```buffer_size``` refer to global variables, next insert the readed value stored in ```msg.data``` in the buffer, removing the oldest data if the buffer is full.  
```
rospy.init_node('reader')
sub = rospy.Subscriber('data_topic', Int32, read_callback)
```
After initializing the node, subscribe the node to the ```data_topic``` and hook to it the ```read_calback``` function.
```
rospy.spin()
```
is equivalent to
```
while not rospy.is_shutdown():
    rate.sleep()
```
and is used when a node doesn't do anything outside of its callbacks.  
For run nodes add before execution permits
```
chmod u+x writer_node.py reader_node.py
```
Run the writer:
```
$ rosrun test_pkg writer_node.py
```
and in another terminal the reader:
```
$ rosrun test_pkg reader_node.py 
[7]
[7, 5]
[7, 5, 4]
[7, 5, 4, 5]
[7, 5, 4, 5, 5]
[5, 4, 5, 5, 5]
[4, 5, 5, 5, 3]
[5, 5, 5, 3, 3]
[5, 5, 3, 3, 6]
[5, 3, 3, 6, 5]
[3, 3, 6, 5, 5]
```
>  :heavy_exclamation_mark: TOPIC QUEUE  
> Every messages published on a topic are stored in a queue and are processed in order by the subscribers. In case the publisher sending the messages at a higher rate than the subsriber nodes can receive them, is possible to drop any messages beyond the size of the queue. When create the publisher set ```queue_size``` to 1 to store only the leatest message
> ```python
> pub = rospy.Publisher('TopicName', CustomMessage, queue_size=1)
>```

### Custom Message
ROS offers a rich set of built-in message types. The ```std_msgs``` package define the primitive types.  
ROS type      | C++ type          | Python type
------------- | ----------------- | -----------
```bool```    | ```uint8_t```     | ```bool```
```int8```    | ```int8_t```      | ```int```
```uint8```   | ```uint8_t```     | ```int```
```int16```   | ```int16_t```     | ```int```
```uint16```  | ```uint16_t```    | ```int```
```int32```   | ```int32_t```     | ```int```
```uint32```  | ```uint32_t```    | ```int```
```int64```   | ```int64_t```     | ```int```
```uint64```  | ```uint64_t```    | ```int```
```float32``` | ```float```       | ```float```
```float64``` | ```double```      | ```float```
```string```  | ```std::string``` | ```string```
```time```    | ```ros::Time```   | ```rospy.Time```

Arrays of this types can be interpreted them as tuple or list in Python. There are also other packages (such as ```geometry_msgs```,```sensor_msgs```) that offers many useful built-in messages that extends ROS standard messages defining structured messages as a composition of primitive types.  
For define a custom message user have to edit a _definition-file_ in the ```msg/``` directory of a package; ```*.msg``` files are simple text files for specifying the data structure of a message.
```
# CustomMessage.msg
string name
float32 value
```
After the the creation of a custom message is needed to modify ```CMakeList.txt``` and ```package.xml``` to allow ROS to generate the language-specific message code. At ```package.xml``` add the lines
```xml 
<build_depend>message_generation</build_depend>

<run_depend>message_runtime</run_depend> 
```
in the ```CMakeList.txt``` file uncomment and modify the following lines:
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation # Add message_generation here, after the other packages
)
#...
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  CustomMessage.msg
)
#...
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
#...
catkin_package(
  CATKIN_DEPENDS message_runtime
```
After this, use ```catkin_make``` from ```~/ros_ws/``` and write a node that use the custom message.
```python
#!/usr/bin/env python3
import rospy
import random
from test_pkg.msg import CustomMessage

rospy.init_node('pub_node')
pub = rospy.Publisher('custom_topic', CustomMessage)
rate = rospy.Rate(0.5)
count = 0
while not rospy.is_shutdown():
	msg = CustomMessage()
	msg.name = 'CustomMessage_'+str(count)    
	msg.value = random.random()
	pub.publish(msg)
    count += 1
	rate.sleep()
```
Run ```roscore```, add execution permits and run ```custom_node.py```, finally display the published messages using
```
$ rostopic echo /custom_topic
name: "CustomMessage_0"
value: 0.376212626696
---
name: "CustomMessage_1"
value: 0.0756872668862
---
name: "CustomMessage_2"
value: 0.0291156787425
---
```

### Services
Another way to communication between nodes is using a **service** that are just a _synchronous remote procedure call_ (it allows one node to call a function that executes in another node).  
It's needed to define the inputs and outputs of this function similarly to the way to define a new message.  
Service mechanism excepts a **server node** which provides the service and **client nodes** which
calls the service.  
Inputs and outputs of the service are defined in a _definition-file_ ```*.srv``` located in ```srv/``` directory of a package.
```
# WordCount.srv
string words # input
---
uint32 count # output
```
Like custom messages, services also require to edit ```package.xml``` and ```CMakeList.txt``` files before build the package. At ```package.xml``` add the lines
```xml 
<build_depend>message_generation</build_depend>

<run_depend>message_runtime</run_depend> 
```
in the ```CMakeList.txt``` file uncomment and modify the following lines:
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation # Add message_generation here, after the other packages
)
#...
# Generate services in the 'srv' folder
add_service_files(
  FILES
  WordCount.srv
)
#...
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
#...
catkin_package(
  CATKIN_DEPENDS message_runtime
```
After this modifies, running ```catkin_make``` will generate three classes: ```WordCount```,```WordCountRequest```, and ```WordCountResponse```. These classes will be used to interact with the service.

#### Service Server
The **server node**, which provides the service, specifies a _callback_ to deal with the _service request_, and advertises the service.  
```python
#!/usr/bin/env python3
import rospy
from test_pkg.srv import WordCount,WordCountResponse

def count_callback(request):
    return WordCountResponse(len(request.words.split()))

rospy.init_node('service_server_node')
rospy.Service('word_count', WordCount, count_callback)
rospy.spin()
```
The server have to include the service message and the service response classes
```python
import rospy
from test_pkg.srv import WordCount,WordCountResponse
```
Next, define the _callback_ that will be invoked when a service is called
```python
def count_callback(request):
    return WordCountResponse(len(request.words.split()))
```
that, in this example, returns the number of word in the sentence separated by a white space.
```python
rospy.init_node('service_server_node')
```
initialize the server node
```python
rospy.Service('word_count', WordCount, count_callback)
```
create a service named ```word_count``` that receive a ```WordCount``` message hooked to the function ```count_callback```.
```python
rospy.spin()
```
keep alive and waits someone who call the service.  
For run the server node (in ```scripts/``` folder) add him the execution permits and use ```rosrun```. Using
```
$ rosservice list
```
is possible to verify that the service is active. For direct call of the service, type
```
$ rosservice call word_count 'one two three'
count: 3
```
or, if there are many inputs arguments, specify
```
$ rosservice call word_count "words"='one two three'
count: 3
```

#### Service Client
A client is a node that use a _local proxy_ to call the remote function.
```python
#!/usr/bin/env python3
import rospy
from test_pkg.srv import WordCount,WordCountRequest

rospy.init_node('service_client_node')
rospy.wait_for_service('word_count')
word_counter_proxy = rospy.ServiceProxy('word_count', WordCount)
req = WordCountRequest()
req.words = 'one two three'
res = word_counter_proxy(req)
print('The number of words are '+str(res.count))
```
Client node import the service message and the service request classes
```python
import rospy
from test_pkg.srv import WordCount,WordCountRequest
```
Then
```python
rospy.init_node('service_client_node')
```
inizialize the cliente node and check if the service server is active
```python
rospy.wait_for_service('word_count')
```
waiting until a server node that implements the ```word_count``` service will running.
```python
word_counter_proxy = rospy.ServiceProxy('word_count', WordCount)
```
Create a _proxy function_ to call the ```word_count``` service that receive ```WordCount``` message.
```
req = WordCountRequest()
req.words = 'one two three'
```
Create a ```WordCountRequest``` that contains the fields specified in the ```WordCount``` message, and set the ```words``` variable.
```python
res = word_counter_proxy(req)
```
Call the service that is running on the server.
```python
print('The number of words are '+str(res.count))
```
Then print the number of words accessing to the response field using the name specified the service file.

### Actions
Another, more sophisticated method of communication between nodes is the **action** mechanism that uses topics to instaurate an _asynchronous bidirectional communication_. Similar to a service, action is used where the server require longer time to respond after receiving a request (_long-running task_), allowing to obtain intermediate response and the possibility to cancel the task at any time.  
Action are also a server-client mechanism, the first step in creating a new action is to define the **goal**, **result**, and **feedback** message formats in a _definition-file_ ```*.action``` in the ```action/``` directory of the package.  
For example let's to implement an action for a _timer_ entity, which waits for a specified time and return a feedback each second.
```
# Timer.action
float32 duration_time # goal
---
time total_time_elapsed # result
---
time partial_time_elapsed # feedback 
```
Before build with ```catkin_make``` modify the file ```package.xml``` adding the lines
```xml
<build_depend>actionlib_msgs</build_depend>

<run_depend>actionlib_msgs</run_depend> 
```
in the CMakeList.txt file uncomment and modify the following lines:
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs # Add message_generation here, after the other packages
)
#...
# Generate services in the 'srv' folder
add_action_files(
  DIRECTORY action
  FILES
  Timer.action
)
#...
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)
#...
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
```
After the building process are created 7 messages in a subfolder of ```devel/``` for perform the action mechanism: _TimerAction.msg_, _TimerActionFeedback.msg_, _TimerActionGoal.msg_, _TimerActionResult.msg_, _TimerFeedback.msg_,
_TimerGoal.msg_ and _TimerResult.msg_.

#### Action Server
The **action server** receive a _goal_ from a client and perform the task in a callback.
```python
#!/usr/bin/env python3
import rospy
import time
import actionlib
from test_pkg.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def timer_callback(goal):
    start_time = time.time()
    result = TimerResult()
    curr_time = time.time()
    while (curr_time - start_time) < goal.duration_time:
        if server.is_preempt_requested():
            # Cancel goal request received
            result.total_time_elapsed = curr_time - start_time
            server.set_preempted(result,'Timer PREEMPTED')
            return
        feedback = TimerFeedback()
        feedback.partial_time_elapsed = curr_time - start_time
        server.publish_feedback(feedback)
        time.sleep(1)
        curr_time = time.time()

    result.total_time_elapsed = curr_time - start_time
    server.set_succeeded(result,'Timer SUCCEFULLY COMPLETED')

rospy.init_node('timer_action_server_node')
server = actionlib.SimpleActionServer('timer', TimerAction, timer_callback, False)
server.start()
rospy.spin()
```
At first, import the ```actionlib``` library and the messages associated to the _action_ and his _goal_, _result_ and _feedback_.
```python
import rospy
import time
import actionlib
from test_pkg.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback
```
The server implements a callback to perform the required task
```python
def timer_callback(goal):
    start_time = time.time()
    result = TimerResult()
    curr_time = time.time()
    while (curr_time - start_time) < goal.duration_time:
        if server.is_preempt_requested():
            # Cancel goal request received
            result.total_time_elapsed = curr_time - start_time
            server.set_preempted(result,'Timer PREEMPTED')
            return
        feedback = TimerFeedback()
        feedback.partial_time_elapsed = curr_time - start_time
        server.publish_feedback(feedback)
        time.sleep(1)
        curr_time = time.time()

    result.total_time_elapsed = curr_time - start_time
    server.set_succeeded(result,'Timer SUCCEFULLY COMPLETED')
```
The server get the current time when a new goal arrive and create a ```TimerResult``` message which will contains the result at the end of the task.  
The loop iterates each second until the difference between ```start_time``` and ```curr_time``` is smaller then ```duration_time``` setted in the _goal_. Before each cycle the server check if a **preempt request** is active (cancellation of the task), in this case the ```result``` of the task get the elapsed time and the _state_ of the action is setted as _preempted_. Otherwise a ```TimerFeedback``` meaasage is sent to the _client_ with the current elapsed time.  
At the end of the loop, when the _goal_ is reached, the ```result``` get the value of the real time elapsed and the state of the action is setted as _succeeded_.
After the node initialization
```python
rospy.init_node('timer_action_server_node')
server = actionlib.SimpleActionServer('timer', TimerAction, timer_callback, False)
server.start()
rospy.spin()
```
create a server as a ```SimpleActionServer```. The first constructor argument is the server’s name, which will determine the namespace into which its constituent topics will be advertised. The second argument is the type of the action that the server will be handling, and the third is the goal callback. The last argumentis the flag to enable the autostarting of the server.  
Then wait in ```spin``` loop until arrive a goal.

#### Action Client
For use an action a **client** have to send a goal to the the server.
```python
#!/usr/bin/env python3
import rospy
import time
import actionlib
from test_pkg.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def feedback_callback(feedback):
    print('Time elapsed: '+str(round(feedback.partial_time_elapsed,2)))

rospy.init_node('timer_action_client_node')
client = actionlib.SimpleActionClient('timer', TimerAction)
client.wait_for_server()
goal = TimerGoal()
goal.duration_time = 10.0
client.send_goal(goal, feedback_cb=feedback_callback)
client.wait_for_result()
print('Time elapsed: '+str(client.get_result()))
```
Client also import action messages and ```actionlib``` package
```python
import rospy
import time
import actionlib
from test_pkg.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback
```
then implements a callback function which will hooked to feedback messages incoming from server
```python
def feedback_callback(feedback):
    print('Time elapsed: '+str(round(feedback.partial_time_elapsed,2)))
```
Initialize the client node
```python
rospy.init_node('timer_action_client_node')
```
create an _action client_ with the same _namespace_ of the server
```python
client = actionlib.SimpleActionClient('timer', TimerAction)
```
then wait any server that implements the specified action is started
```python
client.wait_for_server()
```
Next
```python
goal = TimerGoal()
goal.duration_time = 10.0
client.send_goal(goal, feedback_cb=feedback_callback)
```
create a ```TimerGoal``` message for wait 10 seconds and send to the server, specifying the name of the callback hooked to the feedback using the key argument ```feedback_cb```.
```python
client.wait_for_result()
```
wait until the action is running, then get the action result using the ```client``` method ```get_result```.
Adding the following lines in the ```feedback_callback``` of the  ```timer_action_client_node```
```python
if feedback.partial_time_elapsed >= 5:
    client.cancel_all_goals()
```
a _cancel request_ is send to the server and the ```timer_action_server_node```  receive a _preempt request_ stopping his task.  
For test the action, run ```roscore```, server and client nodes.

## Parameters Server
The parameter in ROS refers to parameters used in the node. Think of it as ```*.ini ``` configuration files. Default values are set in the parameter file and can be read or written if necessary. In particular, it is very useful when configured values can be modified in real-time. Parameters can be setted in a ```*.yaml``` file and can be read from the node or modified during the execution.  
Referring at the first example with the _reader_ and _writer_ nodes, is possible to set the parameters of the nodes in a ```*.yaml``` configuration file
```xml
reader: {'buffer_len':3}
writer: {'freq':1, 'min':3, 'max':7}
```
and modify the node using the ```get_param``` command for get the value of the parameters.
```python
# reader_node.py
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

global buffer, buffer_size
buffer_size = rospy.get_param('/reader/buffer_len',5)
buffer = []

def read_callback(msg):
    global buffer, buffer_size
    if len(buffer) == buffer_size:
        buffer.pop(0)
    buffer.append(msg.data)
    print(buffer)

rospy.init_node('reader')
sub = rospy.Subscriber('data_topic', Int32, read_callback)
rospy.spin()
```
```python
# writer_node.py
#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Int32

rospy.init_node('writer')
pub = rospy.Publisher('data_topic', Int32)
freq = rospy.get_param('/writer/freq',1)
min_value = rospy.get_param('/writer/min',1)
max_value = rospy.get_param('/writer/max',10)
rate = rospy.Rate(freq)
while not rospy.is_shutdown():
    pub.publish(random.randint(min_value,max_value))
    rate.sleep()
```
In the ```get_param``` function of ```rospy``` the first argument is the path of name which the parameter is tored and the optional second argument is the default value if the parameter is unsetted in the ```*.yaml``` file.  
To include the parameters file when start the node see the next section about _launch files_.

## Launch Files
ROS provides a tool to execute concurrently more than one node: ```roslaunch```. This command use ```*.launch``` files (which are XML-based and provides tag-specific options) to specify which node will execute. 
Referring at the first example with the _reader_ and _writer_ nodes, they can be execute by the following _launch-file_ (conventionally located in the ```launch``` folder)
```xml
<launch>
    <node pkg="test_pkg" type="reader_node.py" name="reader" output="screen"/>
    <node pkg="test_pkg" type="writer_node.py" name="writer" />
</launch>
```
A tag ```<node>``` is use for each node which will execute, specifying the package ```pkg```, the name of the script ```type``` and the name of the node ```name```; the ```output``` parameter setted to _"screen"_ is used to display the printed values from the _reader_ node.  
Using the command in the linux shell
```bash
$ roslaunch test_pkg multinodes.launch
```
the two nodes will running, this can be tested typing
```bash
$ rosnode list
/reader
/rosout
/writer
```
and
```bash
$ rostopic list
/data_topic
/rosout
/rosout_agg
```
Launch file allow also to rename node parameters or topic name using the ```<remap>``` tag, specifying the old name and the new name:
```xml
<launch>
    <node pkg="test_pkg" type="reader_node.py" name="reader" output="screen">
        <remap from="data_topic" to="data"/>
    </node>
    <node pkg="test_pkg" type="writer_node.py" name="writer">
        <remap from="data_topic" to="data"/>
    </node>
</launch>
```
Which can be verified typing
```bash
$ rostopic list
/data
/rosout
/rosout_agg
```
In the launch files is also possible to include the parameter file saved as ```*.yaml``` adding the tag ```<rosparam>```.
The previous file can be modified to include the ```param.yaml``` file contained in the ```param``` folder:
```xml
<launch>
	<rosparam command="load" file="$(find test_pkg)/param/param.yaml" />

	<node pkg="test_pkg" type="reader_node.py" name="reader" output="screen">
        <remap from="data_topic" to="data"/>
    </node>
    <node pkg="test_pkg" type="writer_node.py" name="writer">
        <remap from="data_topic" to="data"/>
    </node>
</launch>
```