 # [MiRo Training](https://emarolab.github.io/MiRo-training/)

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
* **"Let's go out"** - The robot leaves the charge to the user that can control its body movement with gestures. The gesture to use are summarized [here](https://ibb.co/6nLxgjw)
* **"Play"** - The robot "follows" the movement of a red ball.
* **"Kill"** - The robot becomes angry. It lights up in red and produces pirate sounds.
* **"Sleep"** - The robot goes in a resting mode. It disables the activation command. Hence, it is not able anymore to execute the other commands until a new command "Miro" wakes it up.

 ## [The Software Architecture](https://ibb.co/4MggTyw)
 The Software architecture can be seen by clicking on the section title. 
 It shows the ROS node (blocks) and the rostopic (blue) used by the nodes to communicate.

 Each module part of the architecture has been implemented as a ROS node. To make communication between the nodes possible, a Publish/Subscribe messaging pattern has been used.
 The Architecture has been organized to be modular, scalable and reusable in few steps.
 The characteristics of each node will be analysed more deeply in the following.
### Description of the Modules

* **command_recognition.py**: this node handles the vocal commands and decides which of them MiRo has to be execute, by publishing on the */miro/rob01/platform/control* topic. In order to be reactive to the instructions, MiRo has to be awakened through the vocal command "Miro".
* **gbb_miro.py**: this node is execute with the vocal command "Let's go out" and handles the gesture based behaviour of Miro. The smartwatch publishes the IMU data on the *\inertial* topic. The **imu_data_map.py** node reads the accelerations' values from the */inertial* topic and maps them into linear and angular velocities, publishing the resulting data on the */imu_mapping* topic. The **gbb_miro.py** node implements the gesture based behaviour in which the robot follows the user's gestures. It publishes the linear and angular velocities on the */gbb* topic.
* **play.py**: this node is executed with the vocal command "Play" and publishes on the *\miro_play* topic. MiRo responds by following the red ball that the user shows him. The right and left camera of MiRo are used for the ball detection. The **hsv_color_filter** node performs the color segmentation of the image obtained from the cameras and publishes on the */right-left/hsv_color_filter/image*. The segmented image is acquired by the **hough_circles** that detects round object, highlighting it with a red circle. If the red ball is detected in the right/left camera MiRo starts turning right/left, until the object is contained in both the cameras. At this point the robot starts moving towards the ball and when it reaches it, it stops.
* **bad.py**: this node is executed with the vocal command "Bad" and publishes on the */miro_bad* topic. MiRo responds to the reproach with a sad and offended behaviour. It lowers and inclines the head, rotates the ears and rotates to its left, showing the back to the user. At the end of the action the leds light up in red, expressing its upset state.
* **kill.py**: this node is executed with the vocal command "Kill" and publishes on the *\miro_kill*topic. MiRo responds with a angry behavior where it lights up in red and emitts pirate sounds.
* **good.py**: this node is executed with the vocal command "Good" and publishes on the *\miro_good* topic. MiRo tries to capture the user's attention by raising up his head and wagging the tail. In this mode the user can touch MiRo on the head or on the body, generating two different behaviours that depend on the values of the sensors published by the */miro_rob01_platform_sensors* topic. When the sensors of the head are activated the robot stops the tail, lowers and inclines the head, squint the eyes and the leds light up in pink. When the sensors on the body are activated the robot stops the tail, raises up the head, rotates the ears and the leds light up in orange. 
* **sleep.py**: this node is executed with the vocal command "Sleep" and publishes on the *\miro_sleep* topic. MiRo responds by entering in a rest mode. It lowers and inclines its head and tail, closes the eyes and the leds light up in aquamarine. In this mode the other vocal commands have no effect, unless it is activated with a new vocal command "MiRo"

 

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

In order to vocally interact with the robot we use a repository that contains an example for using a web interface to speak with the robot. It is based on Google Speech Demo for performing speech-to-text. We disabled the text-to-speech functionality.

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
$ rostopic echo \inertial
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

Click the picture below for demostration video.

[![MiRo-Training - SoRo](https://img.youtube.com/vi/DoKFgs3enpU/0.jpg)](https://www.youtube.com/watch?v=DoKFgs3enpU&feature=youtu.be).


Each partecipant was asked to fill the [questionaire](http://bit.ly/MiroTrainingSurvey) in order to evaluate the interaction with the robot.
## Recommendations
1 ) Once cloned the repository [ros_verbal_interaction_node](https://github.com/EmaroLab/ros_verbal_interaction_node), be sure to unsubscribe from the topic */text_to_speech*. We brutally modified the speech_web_interface.html file by publishing to an unusubscribed topic insted to */text_to_speech*. This to avoid the web interface to repeat what just said by the user.


## Acknowledgments

* [mqtt_ros_bridge](https://github.com/EmaroLab/mqtt_ros_bridge) 
* [imu_stream](https://github.com/EmaroLab/imu_stream)


### Team
* Roberta Delrio *roberta.delrio@studio.unibo.it*
* Valentina Pericu *valentina.pericu.@gmail.com*
