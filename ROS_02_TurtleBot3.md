# TURTLEBOT 3
Robots supported by ROS can be found on the Wiki page (http://robots.ros.org/). Over 200 robots are developed based on ROS. Some of these include custom robots that are publicly released by developers, and it is a noticeable list considering a single system supports such diverse robots. One of the most well-known robots among these is the **TurtleBot**.  
**Turtlebot 3** is a small, programmable, ROS-based mobile robot very used in education, research and prototyping. There are three official model: TurtleBot3 Burger, Waffle, Waffle Pi.  
![TurtleBot 3 models](img/turtlebot_01.png)
Both hardware and software (also ```*.stl``` files of the 3D models) are open-source and distribuited on cloud by the _ROBOTIS Co. Ltd_, so users can modify and customize their platform for any purpose.

## Hardware
Turtlebot 3 is a differential drive based on _Dynamixel_ actuators for move the wheels, a _single-board computer_ for opetrating with ROS (Raspbarry Pi or Intel Joule), an _embedded board_ based on _Cortex-M7_ microcontroller used as sub-controller, and can be equipped with various sensors such as imu, depth camera for 3D recognition, LiDaR, etc.
![TurtleBot 3 equipment](img/turtlebot_02.png)

ROS requires an operating system such as Linux for run, but conventional O.S. does not guarantee real-time operation, and microcontrollers suitable for real-time control are required to control actuators and sensors.
TurtleBot3 use a ARM Cortex-M7 series microcontroller placed on the **OpenCR** (_Open-source Control Module for ROS_) board for performs this low level task.  
OpenCR board use the _STM32F746_ chip as main MCU, and provides _Arduino UNO_ compatible interface, so various libraries, source code and shield modules made for
Arduino development environment can be used.  
OpenCR board supports the main communication interfaces such as UART, I2C, SPI, CAN, TTL, RS485, and includes _MPU925010_ chip, which is integrated triple-axis gyroscope, triple-axis accelerometer, and triple-axis magnetometer sensor in one chip, therefore, various applications using IMU sensor can be used without adding a sensor.  
OpenCR provides a firmware to communicate with the peripherals that can be download directly by the Arduino IDE.
  
## Software
The Turtlebot 3 software consists of a firmware for the embedded board used as sub-controller and 4 ROS packages.  
The firmware is also knowed ```turtlebot3_core``` and is loaded on the embedded board located on the real platform. It provides the management of the motors, the reading of the encoder values and the estimation of the robot location, collecting also the values from the imu.  
For use ROS topic/messages communication mechanism on a embedded system is possible to use ```rosserial```, that is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a serial port. ```rosserial``` works as client/server mechanism to exchange messages between the single-board PC and the microcontroller: the PC running ROS is the server and the microcontroller becomes a client.
![Rosserial protocol](img/rosserial.png)

The server is performed using a ROS node related to the language used (refers to ROS packages ```rosserial_python```, ```rosserial_server``` for C++, ```rosserial_java```), whereas the client library supports all _Arduino platforms_ or _mbed platforms_.  
The rosserial server and client send and receive data in packets based on serial communication. The rosserial protocol is defined in byte level and contains information for packet synchronization and data validation.
![Rosserial packet](img/rosserial_packet.png)

The rosserial packet includes the header field to send and receive the ROS standard message and the checksum field to verify the validity of the data:
- **Sync Flag**: this flag byte is always 0xFF and indicates the start of the packet.
- **Sync Flag / Protocol version**: this field indicates the protocol version of ROS where Groovy is 0xFF and Hydro, Indigo, Jade, Kinetic, Melodic are 0xFE.
- **Message Length**: this 2 bytes field indicates the data length of the message transmitted through the packet. The Low byte comes first, followed by the High byte.
- **Checksum over message length**: the checksum verifies the validity of the message length and is calculated as  
```255 - ((message_length_low_byte + message_length_high_byte) % 256)```
- **Topic ID**: the ID field consists of 2 bytes and is used as an identifier to distinguish the message type. Topic IDs from 0 to 100 are reserved for system functions. The main topic IDs used by the system are shown below.  
```uint16 ID_PUBLISHER=0```  
```uint16 ID_SUBSCRIBER=1```  
```uint16 ID_SERVICE_SERVER=2```  
```uint16 ID_SERVICE_CLIENT=4```  
```uint16 ID_PARAMETER_REQUEST=6```  
```uint16 ID_LOG=7```  
```uint16 ID_TIME=10```  
```uint16 ID_TX_STOP=11 ```
- **Serialized Message Data**: this data field contains the serialized messages.
- **Checksum over topic ID and Message Data**: this checksum is for validating Topic ID and message data, and is calculated as follows.  
```255 - ((topic_ID_low_byte + topic_ID_high_byte + data_byte_values) % 256)```
- **Query Packet**: when the rosserial server starts, it requests information such as topic name and type to the client. When requesting information, the query packet is used. The Topic ID of query packet is 0 and the data size is 0. The data in the query packet is shown below.  
```0xff 0xfe 0x00 0x00 0xff 0x00 0x00 0xff```  
When the client receives the query packet, it sends a message to the server with the following data, and the server sends and receives messages based on this information.  
```uint16 topic_id```  
```string topic_name```  
```string message_type```  
```string md5sum```  
```int32 buffer_size```  

> #### :warning: ROSSERIAL LIMITATION
> With ```rosserial``` is possible to send and receive ROS standard messages with the embedded system using the UART protocol, but there are some hardware limitations of the embedded system that may cause an
issue. For example, memory constraints (more then 25 topic can be arise some issue) or ```Float64``` and ```String``` type, that are not supported by several microcontroller.

Whereas, the ROS packages running on the PC are organized as follow:
- ```turtlebot3```: contains Turtlebot's robot models, SLAM and navigation package, remote control package and bringup package.
- ```turtlebot3_msgs```: contains message files used in ```turtlebot3```.
- ```turtlebot3_simulations```: contains packages used for simulation in Gazebo.
- ```turtlebot3_applications```: provides some application examples with the Turtlebot 3.

## TurtleBot3 Install
#### Install dependent packages
Install dependent packages in ROS main folder: ```/opt/ros/melodic```.
```bash
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

#### Install TurtleBot3 packages
Install TurtleBot3 packages in the workspace, from the official github repository https://github.com/ROBOTIS-GIT.
```bash
$ cd ~/ros_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ..
$ catkin_make
```
>##### :children_crossing: BUILDING PROBLEM
>If there are problems in building phase with python _em_ module install the library:
> ```
> $ python -m pip install empy
> $ python3 -m pip install empy 
> ```

## Simulation
For work in simulation environment with Gazebo and Rviz is needed to choose which model of Turtlebot we want to use (choose between ```burger```,```waffle``` and ```waffle_pi```), setting the environment variable as follow
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
```
> ####  :fast_forward: SET ENVIRONMENT VARIABLE
> For improve the working flow, it's possible to include the setting of this environment variable in the ```.bashrc``` file, adding the following lines:
> ```
> # Export Robot Model
> export TURTLEBOT3_MODEL=waffle_pi
> ```
> This allows not having to specify this setting in each new terminal.

### Spawn model in Gazebo
For run the Gazebo simulation environment, type
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```
or 
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```
in the ```turtlebot3_description``` package there are the files that describes the Turtlebot by the urdf, the Gazebo specifications for physical simulations and the meshes files used for modelling the parts of the robot. The ```turtlebot3_gazebo``` package mainly contains the launch file for spawn the robot model in different Gazebo scenarios.

### Teleop Turtlebot 3
For teleoperate the Turtlebot is available a node in the ```turtlebot3_teleop``` package for send the velocities commands on the ```cmd_vel``` topic, reading from the keyboard. Type in a terminal:
```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Visualize Turtlebot 3 on Rviz
For visualize the belief of the robot, such as his position, his configuration or the data information from his sensors, advertised on the topic, run Rviz using the ```*.launch``` file
```bash
$ roslaunch turtlebot3_gazebo turtlebort3_gazebo_rviz.launch
```
or manually start Rviz and select which information would to display adding the related object
```bash
$ rviz
```
If you choose to start Rviz standalone, you have to run the ```robot_state_publisher``` from the homonym package
```bash
$ rusrun robot_state_publisher robot_state_publisher
```
Depending on the Turtlebot model chosen, it's possible to adding different sensor on Rviz, for know what sensor are equipped on your model, browse the related```*.gazebo.xacro``` file in the ```turtlebot3_description``` package.  
For example, using the Turtlebot 3 Waffle Pi, it's possible to get environmental information from the LiDaR and from a RGB Camera. After adding the robot model, try to add the sensors using ```LaserScan``` and ```Camera``` object, next select the ```/scan``` topic for the laser and the ```/camera/rgb/image_raw``` for the camera.  

![TurtleBot 3 in Rviz](img/turtlebot_rviz.png)

## Build a Map
Very useful aspect to perform autonoumus navigation is build a map of the environment in which the robot moves.  
Navigation maps in ROS are represented by a 2D grid, where each grid cell contains a value that corresponds to how likely it is to be occupied: white is open space, black is occupied, and the grayish color is unknown. Map files are stored as image (```*.png```,```*.jpg```, ```*.pmg```). Associated with each map is a
```*.yaml``` file that holds additional information, such as the resolution (the length of each grid cell in meters), where the origin of the map is, and thresholds for deciding if a cell is occupied or unoccupied.  
To build a map, ROS makes available several packages (such as ```gmapping```), that record data from sensors and simultaneously localize the robot in the enviorment reading the odometry informations. For construct a good map is needed to perform slow movement (in particular the turn movement) and cover the whole area many times.
Using the ```slam_gmapping``` node from ```gmapping``` package, it's possible to build the map of the environment.

![Mapping process on the TurtleBot 3](img/mapping_process.png)

Before run the ```slam_gmapping``` node set the dimension of the map that you would to build specifying in the parameter server the following parameters:
```bash
$ rosparam set /slam_gmapping/xmax 10  
$ rosparam set /slam_gmapping/xmin -10
$ rosparam set /slam_gmapping/ymax 10  
$ rosparam set /slam_gmapping/ymin -10
```
then run the node
```bash
$ rosrun gmapping slam_gmapping
```
Try now to add a display of type _Map_ on Rviz, and set the topic name to ```/map```. Make sure that the fixed
frame is also set to ```/map```. After this try to move the Turtlebot around the world with the teleoperation node and see in Rviz the building process of the map.  

![Building a map](img/turtlebot_map.png)

When you think that the builded map is aceptable, use the ```map_saver``` node from the ```map_server``` package for save the map and his configuration file on the disk, in another terminal types:
```bash
$ rosrun map_server map_saver
```
and see the two files saved in your user's home folder. The ```map_saver``` node generates an image that contains the map and a ```*.yaml``` file that contains the information about the map resolution, the coordinates frame and the treshold values for classify free, occuped or unknown grid cells.  
```
image: map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
Move this files in a ```map/``` folder in the package for use it in autonomous navigation.  
You can use the ```gmapping``` package also on your own robot to reconstruct a map of your custom environment.

![Building a map with own robot](img/smr_map.png)
