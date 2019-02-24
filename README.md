 # MiRo Training

 ## The Project
 This Project has been developed for the Social Robotics course of the master degree program in Robotics Engineering at University of Genoa.

 ### MiRo Companion Robot
 MiRo is a animal-like robot developed as a prototype companion.
 It was designed with a bio-inspired architecture based on the neuroscience knowledge of the mammalian brain.

 ### The Objective
The aim of the project is to develop a software architecture for interacting with Miro using vocal and gestural commands.
The robot attention is obtained through the vocal activation command **"Miro"**.
Only after the activation command the robot is able to execute further commands.
The possibilities are the following:
* **"Good"** - The robot expresses a cheerful and noisy behaviour and requires to be touched by the user to calm down.
* **"Bad"** - The robot becomes upsed for being scolded and turn its back to the user.
* **"Let's go out"** - The robot leaves the charge to the user that can control its body movement with gestures.
* **"Play"** - The robot "follows" the movement of a red ball.
* **"Sleep"** - The robot goes in a resting mode. It disables the activation command. Hence, it is not able anymore to execute the other commands until a new command "Miro" wakes it up.



 ## The architecture

### Overall Architecture

### Description of the Modules

 

 # Gettin Started

 ## Prerequisites

 ### ROS
This project is developed using [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu):
* rosdistro: kinetic
* rosversion: 1.12.13

### MiRo Workstation Setup
Download the [Miro Developer kit](http://labs.consequentialrobotics.com/miro/mdk/).

Follow the instructions from Consequential Robotics [Miro: Prepare Workstation](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html) to set up your workstation to work with mthe robot. 
Strictly follow the instructions in the Install **mdk** section as the following steps will rely on this.
Not necessary to make static IP for your workstation (laptop) while setting up connection with MiRo.
For a clear tutorial step-by-step you should visit [Emarolab Miro Repository](https://github.com/EmaroLab/MIRO.git).

 ### The wearable device
 In order to interact with MiRo through gestures, a smartwatch with a 9-axis IMU sensor has been used.
 [LG G WATCH R](https://www.lg.com/wearable-technology/lg-G-Watch-R-W110)
Follow the instructions reported in [imu_stream](https://github.com/EmaroLab/imu_stream) to download the app for both the smartphone and the smartwatch.

### Smartwatch and Smartphone Setup
In order to publish imu sensor data from your smartwatch to ROS nodes you must have a smartwatch paired with a smartphone.
The smartphone acts as the bridge between the smartwatch and the ros master running on your computer.

### MQTT ROS Bridge

In order to succesfully subscribe to MQTT topics and publish contents of MQTT messages to ROS follow the instruction in [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge/tree/feature/multiple_smartwatches).
To work with the current project some parameter must be modified in the imu_bridge.launch 
The parameter device_name must be changed with the name of your personal smartwatch. 


### ROS Based Speech Interface

In order to vocally interact with the robot we use a repository that contains an example for using a web interface to speak with the robot. It is based on Google Speech Demo for performing speech-to-text. We disabled the text-to-speech functionality ( look at Recommendations section to see how we did it).

For this project we used the mic in [LOGITECH Wireless Headset H600](https://www.logitech.com/it-it/product/wireless-headset-h600), but any microphone connected to your laptop should work pretty fine.

Create a catkin workspace and clone all the packages in the src folder

```
$ git clone https://github.com/EmaroLab/ros_verbal_interaction_node.git

```

For further information follow the instruction contained in [ros_verbal_interaction_node](https://github.com/EmaroLab/ros_verbal_interaction_node) repository.

### OpenCV apps 

The images streaming from Miro's camera are processed using the package [opencv_apps](http://wiki.ros.org/opencv_apps).
The camera frames are subject to color segmentation and hugh circles detection.
To install it:
```
$ git clone sudo apt install ros-kinetic-opencv-apps

```

### MiRo Training

Create a catkin workspace and clone all the packages in the src folder

```
$ git clone https://github.com/EmaroLab/MiRo-training.git
$ catkin_make
$ source devel/setup.bash
```

## Run the Project

Open a new terminal and launch

```
$ roscore
```
mosquitto must be running on your PC for the birdge to work.

In a new terminal
```
$ mosquitto
```
Make sure that the IP in the IMU_stream app on the smartphone is the same shown by doing

```
$ ifconfig
```

Open the IMU_stream app on the smartwatch 
To test if the connection between smartwatch and ROS is working, start to transmitt the data from IMU_stream app on the smartwatch and check in a new terminal
```
$ rostopic echo \imu_left_hand
```
It should see the Imu data published by the smartwatch.

Connect the Miro robot to the ROS Master

```
$ ssh root@<MIRO-IP> 
$ sudo nano ./profile
```
Insert your IP after ROS_MASTER_IP

For more detailed instructions see [MIRO: Commission MIRO](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)

Open in a new terminal your catkin_ws
The following command will start the project

```
$ roslaunch command_handler command_handler.launch 

```

Parameters that is possible to change directly from the launch file:
* Robot --> sim01 or rob01 (Default value)
	To switch from real robot to simulation ( You should launch Gazebo with miro sim as explained in [MIRO: Consequential Robotics](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html)
* Node rate --> 200 Hz (Default value)
* Color of the ball to detect --> 
	h_limit_min = 0; h_limit_max = 10; s_limit_min = 255; s_limit_max = 150; v_limit_min = 255; v_limit_max = 50;
	Change the min and maximum HSV values to detect different colors.
	To discover the HSV values of your favorite color, check [this](https://alloyui.com/examples/color-picker/hsv).
## Results

## Recommendations


## Acknowledgments

* [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge) 
* [imu_stream](https://github.com/EmaroLab/imu_stream)


### Team
* Roberta Delrio *roberta.delrio@studio.unibo.it*
* Valentina Pericu *valentina.pericu.@gmail.com*
