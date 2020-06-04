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

## COMPUTER VISION
Another common example of robotics application (especially for mobile robotics) involves the **computer vision** field. _Computer vision_ is a branch of the **artificial intelligence**, its tasks include methods for acquiring, processing, analyzing and understanding digital images, and extraction of high-dimensional data from the real world in order to produce numerical or symbolic information used for take decisions.  
The purpose of computer vision is 
> "Make the robot (or, in general, the machines) able to see the environment as human visual system can do."

Computer acquire images like a set of numbers, any image consists of a set of _pixel_, that is the elementary unit of the image. The pixels total number is called _resolution_ of the image, so if an image can be display like a matrix with 600x480 cells, the total number of pixel is 288.000. A single pixel contains the information about the color of a specific cell, that may be a number in the interval [0;255] for only grayscale image, or a tuple of three values for a color image, where each value represent the intensity of one of the primary color: red, green, blue (RGB).

![Primary Color](img/colors.png)

Therefore, a grayscale image can be represent like a 2D matrix where each pixel is the intensity of the darkness: 0 is black, 255 is white. For a colored image the 2D matrix are three, one for each color channel.

### OpenCV
**OpenCV** (_Open Source Computer Vision Library_) is an open source computer vision and machine learning software library. OpenCV was built to provide a common infrastructure for image manipulation and advanced computer vision applications. The library has more than 2500 optimized algorithms, which includes a comprehensive set of both classic and state-of-the-art computer vision and machine learning algorithms. These algorithms can be used to detect and recognize faces, identify objects, classify human actions in videos, track camera movements, track moving objects, extract 3D models of objects, produce 3D point clouds from stereo cameras, stitch images together to produce a high resolution image of an entire scene, find similar images from an image database, etc.  
It has C++, Python, Java and MATLAB interfaces and supports Windows, Linux, Android and Mac OS.

#### Install OpenCV
OpenCV source code can be downloaded from the official [web site](https://opencv.org/releases/) or from its [github's repository](https://github.com/opencv/opencv/tree/4.3.0), alternatively is possible to install the unofficial pre-built OpenCV packages for Python using ```pip```.
For Python 2 or Python 3, type
```bash
$ pip install opencv-contrib-python
```
or
```bash
$ pip3 install opencv-contrib-python
```

#### Using OpenCV with Python
Let's to see some simple example to read, manipulate and display images with OpenCV, using Python scripts.  
OpenCV can read an image from a file, and offers support for the main image formats, such as: ```png```, ```jpg```, ```pgm```, ```bmp```, ```tif```, etc.
```python
import cv2

# Load image
img = cv2.imread('colors_triangle.jpg')

# Display image
cv2.imshow('ORIGINAL IMAGE',img)
cv2.waitKey()
cv2.destroyAllWindows()
```
As said before, a color image can be splitted in three 2D matrix, one for each color channel. When the ```cv2.imread``` function is invoked, the image is saved with a BGR color format, so the first matrix is for the blue color, the second for green, and the last for red. So, is possible to extract the intensity matrix for each color using the python slicing
```python
b = img[:,:,0]
g = img[:,:,1]
r = img[:,:,2]
```
or the ```cv2.split``` function
```python
b,g,r = cv2.split(img)
```
![Color channels](img/color_filter.png)

OpenCV allows to modify the single pixel of the image or copy and paste a part of its in another place. Also, OpenCV offer a set of function to manipulate the image, like the classic _binarization_: if the pixel value is smaller than the threshold, it is set to 0, otherwise it is set to a maximum value; or using different type of threshold for saturate only the high values or the low values.
```python
# Binary saturation
ret,thresh1 = cv2.threshold(r,64,255,cv2.THRESH_BINARY)
cv2.imshow('BINARY SATURATION',thresh1)
cv2.waitKey()

# Lower saturation
ret,thresh2 = cv2.threshold(r,64,255,cv2.THRESH_TOZERO)
cv2.imshow('BINARY SATURATION',thresh2)
cv2.waitKey()

# Upper saturation
ret,thresh3 = cv2.threshold(r,64,255,cv2.THRESH_TRUNC)
cv2.imshow('BINARY SATURATION',thresh3)
cv2.waitKey()
cv2.destroyAllWindows()
```
The previous examples using one global value as a threshold. But this might not be good in all cases, e.g. if an image has different lighting conditions in different areas. In that case, adaptive thresholding can help. Here, the algorithm determines the threshold for a pixel based on a small region around it. So we get different thresholds for different regions of the same image which gives better results for images with varying illumination.
```python
import cv2

# Load image
img = cv2.imread('beer.jpg')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Display image
cv2.imshow('GRAYSCALE',img_gray)
cv2.waitKey()

# Adaptive median threshold
th1 = cv2.adaptiveThreshold(img_gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
cv2.imshow('ADAPTIVE MEAN THRESHOLD',th1)
cv2.waitKey()

# Adaptive Gaussian threshold
th2 = cv2.adaptiveThreshold(img_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
cv2.imshow('ADAPTIVE GAUSSIAN THRESHOLD',th2)
cv2.waitKey()
cv2.destroyAllWindows()
```

These kind of threshold improve the detection of the edge of the image. Other very useful tool to elaborate image are filters. As in one-dimensional signals, images also can be filtered with various low-pass filters (LPF), high-pass filters (HPF), etc. LPF helps in removing noise, blurring images, etc. HPF filters help in finding edges in images.  
Image Blurring (Image Smoothing) is achieved by convolving the image with a low-pass filter kernel. It is useful for removing noise. It actually removes high frequency content (eg: noise, edges) from the image. This is done by convolving an image with a normalized box filter. It simply takes the average of all the pixels under the kernel area and replaces the central element. This is done by the function ```cv2.blur``` that receives the dimension of the box filter and construct a normalized box, for example, for a 3x3 window:

![K = \frac{1}{9}\begin{bmatrix}1 \,\,\, 1 \,\,\, 1\\1 \,\,\, 1 \,\,\, 1\\1 \,\,\, 1 \,\,\, 1\end{bmatrix}](https://render.githubusercontent.com/render/math?math=K%20%3D%20%5Cfrac%7B1%7D%7B9%7D%5Cbegin%7Bbmatrix%7D1%20%5C%2C%5C%2C%5C%2C%201%20%5C%2C%5C%2C%5C%2C%201%5C%5C1%20%5C%2C%5C%2C%5C%2C%201%20%5C%2C%5C%2C%5C%2C%201%5C%5C1%20%5C%2C%5C%2C%5C%2C%201%20%5C%2C%5C%2C%5C%2C%201%5Cend%7Bbmatrix%7D)

Instead use a box filter, is possible to choose a _Gaussian kernel_ to perform the filtration. This can be done using the ```cv2.GaussianBlur``` function.  

![K = \frac{1}{16}\begin{bmatrix}1 \,\,\, 2 \,\,\, 1\\2 \,\,\, 4 \,\,\, 2\\1 \,\,\, 2 \,\,\, 1\end{bmatrix}](https://render.githubusercontent.com/render/math?math=K%20%3D%20%5Cfrac%7B1%7D%7B16%7D%5Cbegin%7Bbmatrix%7D1%20%5C%2C%5C%2C%5C%2C%202%20%5C%2C%5C%2C%5C%2C%201%5C%5C2%20%5C%2C%5C%2C%5C%2C%204%20%5C%2C%5C%2C%5C%2C%202%5C%5C1%20%5C%2C%5C%2C%5C%2C%202%20%5C%2C%5C%2C%5C%2C%201%5Cend%7Bbmatrix%7D)
```python
# Blurring
img_ab = cv2.blur(img,(5,5))
img_gb = cv2.GaussianBlur(img,(5,5),0)
cv2.imshow('FILTERING',img_ab)
cv2.waitKey()
cv2.imshow('FILTERING',img_gb)
cv2.waitKey()
cv2.destroyAllWindows()
```
Another analysis that can be performed on an image is verify of fast change the intensity of the color (for a grayscale image: measure the gradient of the intensity). This is useful to identify the edges of objects in the pictures. To perform this analysis, OpenCV provides three types of _gradient filter_ (that are HPFs): Sobel, Scharr, Laplacian. Next are proposed some examples about gradient filtration on a grayscale image.
```python
# Gradient
img_laplacian = cv2.Laplacian(img_gray,cv2.CV_8U)
img_sobelx = cv2.Sobel(img_gray,cv2.CV_8U,1,0,ksize=5)
img_sobely = cv2.Sobel(img_gray,cv2.CV_8U,0,1,ksize=5)
img_sobelxy = cv2.Sobel(img_gray,cv2.CV_8U,1,1,ksize=5)
cv2.imshow('EDGE',img_laplacian)
cv2.waitKey()
cv2.imshow('EDGE',img_sobelx)
cv2.waitKey()
cv2.imshow('EDGE',img_sobely)
cv2.waitKey()
cv2.imshow('EDGE',img_sobelxy)
cv2.waitKey()
cv2.destroyAllWindows()
```
The process of edge detection is one of the more used for perform object detection. A very popular method to detect the object edges is the _Canny Detection Alghoritm_. This algorithm involves different stages
- **Noise Reduction**: since edge detection is susceptible to noise in the image, first step is to remove the noise in the image with a 5x5 _Gaussian filter_, as we have already seen in previous example.
- **Compute Intensity Gradient**: smoothened image is then filtered with a _Sobel kernel_ in both horizontal and vertical direction to get first derivative in horizontal direction (Gx) and vertical direction (Gy). From these two images, we can find edge gradient and direction for each pixel.
- **Non-maximum Suppression**: after getting gradient magnitude and direction, a full scan of image is done to remove any unwanted pixels which may not constitute the edge. For this, at every pixel, pixel is checked if it is a local maximum in its neighborhood in the direction of gradient.
- **Hysteresis Thresholding**: this stage decides which are all edges are really edges and which are not. For this, we need two threshold values, minVal and maxVal. Any edges with intensity gradient more than maxVal are sure to be edges and those below minVal are sure to be non-edges, so discarded. Those who lie between these two thresholds are classified edges or non-edges based on their connectivity. If they are connected to "sure-edge" pixels, they are considered to be part of edges. Otherwise, they are also discarded.
```python
# Edge Detection
img_edges = cv2.Canny(img,100,200)
cv2.imshow('EDGE',img_edges)
cv2.waitKey()
cv2.destroyAllWindows()
```
### ROS and OpenCV
To apply image processing to image recorded from a camera on a robot that are using ROS, OpenCV provides a _bridge interface_ that converts ```sensor_msgs/Image``` ROS messages in OpenCV image format.
Next is proposed a simple example to explain how to read an image from a ROS topic, convert it in grayscale with OpenCV, and display the new image on another ROS topic.
```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

class CameraGrayscale:
    def __init__(self):
        rospy.init_node('camera_grayscale')
        # ------ VARIABLES ------
        self.bridge = cv_bridge.CvBridge()
        # ------ TOPICS ------
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image, self.image_callback)
        self.image_pub = rospy.Publisher('camera/rgb/image_gray', Image, queue_size=1)
    
    def loop(self):
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS image in OpenCV image
        img_CV = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        # Convert rgb image in grayscale
        img_gray = cv2.cvtColor(img_CV, cv2.COLOR_BGR2GRAY)
        # Convert OpenCV image in ROS image
        img_ROS = self.bridge.cv2_to_imgmsg(img_gray, encoding='passthrough')
        # Add the camera reference frame
        img_ROS.header.frame_id = 'camera_rgb_frame'
        # Publish the image on the topic
        self.image_pub.publish(img_ROS)

if __name__ == '__main__':
    cg = CameraGrayscale()
    cg.loop()
``` 

### Objects Detection
One of the most required task for an artificial vision system is indentify and recognize what it see. To perform this task are used corner, feature and edge detection algorithm, where this elements are used to matching the image with the objects present in a database.  
These algorithm can be use **machine learning** methodologies, **artificial neural networks** and other state-of-the-art classification techniques.  
The main steps of these methodologies can be summarized in the following:
- Load in a database a dataset of images tied with their labels, that represent how the image is classified.
- Training the artificial intelligence system, building a classifier that search the same features between the object with the same label.
- Analyze a new unknown image and match its features to predict the label of its class.  

We cannot train deep learning models using OpenCV (there are already great frameworks available for that purpose), but we can use pre-trained deep learning models, saving a lot of time in our applications.  
Since the release 3.3 of OpenCV, the deep learning module (```dnn```) was improved, supporting a wide number of deep learning frameworks, including **Caffe**, **TensorFlow**, and **Torch/PyTorch**. This module allows to load a model from disk, pre-process an input image, then pass the image through the neural network and obtain the output classification.  
The detection model that we are using is the ```MobileNetSSD```, trained on the **COCO dataset** (_Common Objects in Context_) and was then fine-tuned on **PASCAL VOC** (_Pascal Visual Object Classes_) reaching 72.7% mAP (mean average precision). 
We can therefore detect 20 objects in images (+1 for the background class), including airplanes, bicycles, birds, boats, bottles, buses, cars, cats, chairs, cows, dining tables, dogs, horses, motorbikes, people, potted plants, sheep, sofas, trains, and tv monitors.  
The trained network is contained in two files: the ```MobileNetSSD_deploy.prototxt.txt``` and the ```MobileNetSSD_deploy.caffemodel```; that can be imported using the ```dnn``` module of OpenCV.  
For each image recorded by the camera on the robot, we process the image through the network and await if one or more object are detected; for each detected object a colored rectangle and its label are displayed around the object.

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        # ------ PARAMETERS ------
        prototxt_file = rospy.get_param('/prototxt', 'MobileNetSSD_deploy.prototxt.txt')
        model_file = rospy.get_param('/model_file', 'MobileNetSSD_deploy.caffemodel')
        self.confidence = rospy.get_param('/confidence', 0.5)

        # initialize the list of class labels MobileNet SSD was trained to
        # detect, then generate a set of bounding box colors for each class
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",\
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",\
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",\
                        "sofa", "train", "tvmonitor"]
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        # load our serialized model from disk
        print("[INFO] loading model...")
        self.net = cv2.dnn.readNetFromCaffe(prototxt_file, model_file)

        self.bridge = cv_bridge.CvBridge()

        # ------ TOPICS ------
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.detect_pub = rospy.Publisher('camera/rgb/object_detect', Image, queue_size=1)
    
    def loop(self):
        rospy.spin()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # load the input image and construct an input blob for the image
        # by resizing to a fixed 300x300 pixels and then normalizing it
        # (note: normalization is done via the authors of the MobileNet SSD
        # implementation)
        (h, w) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
        
        # pass the blob through the network and obtain the detections and
        # predictions
        print("[INFO] computing object detections...")
        self.net.setInput(blob)
        detections = self.net.forward()

        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with the
            # prediction
            confidence = detections[0, 0, i, 2]
            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > self.confidence:
                # extract the index of the class label from the `detections`,
                # then compute the (x, y)-coordinates of the bounding box for
                # the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # display the prediction
                label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                print("[INFO] {}".format(label))
                cv2.rectangle(image, (startX, startY), (endX, endY), self.COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

        # Display the resulting frame
        image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image.header.frame_id = 'camera_rgb_frame'
        self.detect_pub.publish(image)

if __name__ == '__main__':
    od = ObjectDetector()
    od.loop()
```
First import the required library and initialize the node that have to perform the object detection

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
```
Specify the files that we'll use to load the pre-trained network model, and set the ```confidence``` values, that will be used as a threshold to detect an object (0.5 = 50%).
```python
# ------ PARAMETERS ------
prototxt_file = rospy.get_param('/prototxt', 'MobileNetSSD_deploy.prototxt.txt')
model_file = rospy.get_param('/model_file', 'MobileNetSSD_deploy.caffemodel')
self.confidence = rospy.get_param('/confidence', 0.5)
```
Initialize the list of class labels with which the _MobileNetSSD_ was trained to detect, then generate a set of bounding box colors for each class
```python
self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",\
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",\
                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",\
                "sofa", "train", "tvmonitor"]
self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))
```
Load the _Caffe_ network model from the files using the ```dnn``` module
```python
self.net = cv2.dnn.readNetFromCaffe(prototxt_file, model_file)
```
Initialize the ```CvBridge``` to transform the image between ROS and OpenCV
```python
self.bridge = cv_bridge.CvBridge()
```
Finally subscribe the node to the images source topic, that read the frame from a camera
```python
self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
```
and publish on a new topic ```camera/rgb/object_detect``` the camera images with colored box and label around the detected objects 
```python
self.detect_pub = rospy.Publisher('camera/rgb/object_detect', Image, queue_size=1)
```
The life cycle of the node just await that images are published on the ```camera/rgb/image_raw``` topic, then process each frame to discovery some knowed object.  
![Detection process](img/blob_from_images_header.png)

The process of detection requires that each frame are preprocessed (transform the frame in a _blob_) before pass it through the network. The main operation is compute the average pixel intensity across all images in the training set for each of the Red, Green, and Blue channels, then subtract these to each frame. Mean subtraction is used to help combat illumination changes in the input images in our dataset. Then scale the image of a desidered factor, swap the red and blue channel (fix differences between RGB and BGR format) and resize the image in a fixed format (300x300):
```python
blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
```
Pass the blob through the network and obtain the detections and predictions
```python
self.net.setInput(blob)
detections = self.net.forward()
```
Loop over the detections, keeping in mind that multiple objects can be detected in a single image. We also apply a check to the confidence (probability threshold) associated with each detection. If the confidence is high, then we’ll draw the prediction on the image with text and a colored bounding box.
```python
for i in np.arange(0, detections.shape[2]):
    confidence = detections[0, 0, i, 2]
    if confidence > self.confidence:
        idx = int(detections[0, 0, i, 1]) # get the label
```
Then are performed some step to draw a colored box with a label around the detected object
```python
box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
(startX, startY, endX, endY) = box.astype("int")
# display the prediction
label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
print("[INFO] {}".format(label))
cv2.rectangle(image, (startX, startY), (endX, endY), self.COLORS[idx], 2)
y = startY - 15 if startY - 15 > 15 else startY + 15
cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)
```
Finally send the frame with detected object on the ROS topic
```python
image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
image.header.frame_id = 'camera_rgb_frame'
self.detect_pub.publish(image)
```
Try to run this application in a Gazebo world that contains some objects with which the network dataset is pre-trained.  
Run a launch file that run the custom world
```bash
$ roslaunch turtlebot3_app city.launch
```
then, run in a new terminal the ```object_detector``` node
```bash
$ rosrun turtlebot3_app object_detector.py
```
and the ```turtlebot3_teleop_key``` node to teleoperate the Turtlebot around the world. Display on Rviz the topic ```camera/rgb/object_detect``` to visualize the object detected.
