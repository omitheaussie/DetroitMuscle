
# Udacity Self-Driving Car Engineer Nanodegree

## Term 3 Project 3 : System Integration
#### Caption : Where it all came together!

#### Project Writeup

---
This is the 3rd and final project of Term 3 and 13th project overall as part Udacity's [Self Driving Car Engineer Nanodegree](writeup/images/CarNDSyllabus.pdf)

This project involves End-End system integration for Self-driving Car that involves Perception, Planning & Control. The project involves implementing ROS(Robot Operating System) nodes to implement core functionality of the autonomous vehicle system, including traffic light classification, detection,waypoint generation, following & Controls. The code will be first tested in Udacity's Simulator, and once it passes through this stage, the code will be fed to Udacity's real Self driving Car a.k.a. *Carla* - how cool is it!

---
### The Team
This is the project where all the learning over past 8 months of the program is put together. Due to the complexity and the breadth of work involved, Udacity recommends to work in teams for this project. 

We formed a group of 4 from our Cohurt and named our team '**Detroit Muscles**' as all of us belong to the Motor City - Detroit. Each of us worked on a seperate module independently, and after individual modules are built, we worked together to integrate each other and complete the system

Our Team (In alphabetical Order)

|     Name                | Role in the project            |     Github                                |
| ----------------------- | ------------------------------ | ----------------------------------------- |
| Nathan Nick Adams       | Waypoint Loading & Generation  | [Github](https://github.com/xxx)          |
| Omkar Karve (Team Lead) | Traffic Light Classification   | [Github](https://github.com/omitheaussie)      |
| Ravi Kiran Savirigana   | Controls                       | [Github](https://github.com/mymachinelearnings)|
| Rui Li                  | Waypoint Generation & Updation | [Github](https://github.com/kyoleelee/Udacity-SelfDrivingCar)|

---

### Autonomous Vehicle Architecture
At a high level, the autonomous vehicle a.k.a. Self Driving Car's architecture can be shown as below

![Autonomous Vehicle Architecture](writeup/images/AutonomousVehicleArch.png)

The components of a Self Driving Car can be mainly divided into 4 components

##### Sensors
   This consists of the hardware components that gather data about the environment. This includes Camera, Lidar, Radars, IMU(Inertial measurement unit), GPS sensors mounted on the car to name a few. This system basically provides the information about its surroundings to the Vehicle's perception subsystem

##### Perception
   This subsystem consists of software to process sensor data. This processes the data received by different components of the the Sensor subsystem, combines them through sensor fusion techniques and harvests them to converts them to meaningful information. This is where most of the vehicles analysis of the environment takes place. This subsystem can be further divided into `detection` & `localization`.
   
    Detection : The detection subsystem is responsible for understanding the surrounding environment like lane detection, traffic sign & light detection & classification, object detection & tracking and free space detection
    
    Localization : The localization subsystem is responsible for using sensor and map data to determine the vehicle's precise location.
    
    Note that each component of the perception subsystem relies on a different group of sensors.

##### Plannning
   The planning subsystem uses the output from perception for behavior planning and for both short and long range path plan. There are several components of the planning system 
   
    Route Planning : high level path of the vehicle between two points on a map. Ths module uses Map data from Sensor subsystem
    
    Prediction: This module identifies which maneuver other objects on the road might take
    
    Behavior planning : Decides what maneuver our vehicle should take
    
    Trajectory Generation : Plots the precise path(a set of points for next few steps) we'd like our vehicle to follow

##### Control
   The control subsystem ensures that the vehicle follows the path provided by the planning subsystem and sends control commands to the vehicle. The control subsystem may include components such as PID controllers, model predictive controllers, or other type of controllers. subsystem sends acceleration, braking, and steering commands to the vehicle via a Drive-By-Wire(DBW) module which essentially converts electronic signals to physical controls of the vehicle

#### Project Architecture

Udacity's real Self Driving car, Carla, follows the same architecture as mentioned above, and as part of this project, we implemented the Perception, Planning & Control modules. Carla uses ROS as its framework to implement and integrate all the nodes of the autonomous vehicle system. 

The following diagram represents the system architecture showing the ROS nodes and topics used in the project

![System Architecture](writeup/images/ArchOverview.png)

---
### Implementation Details

While the architecture diagram above is self-explanatory, we would like to explain some of the primary nodes here. Each of these is implemented as a ROS node, and implemted in Python 3.6

#### Trafffic Light Detection Node 

This node forms the Perception module and primarily implements 2 functionalities


| Topic Subscription | Description |
| ------------------ | ----------- |
| /base_waypoints    | A map of all the waypoints of the environment |
| /current_pose      | Provides the current stats of the vehicle like current position, linear & angular velocities, etc |
| /image_color       | Provides camera feed |
| /vehicle/traffic_lights | This is actually not used in the final submission. This is like a helper module that can simulate traffic light classification while the classifier is being worked on |

| Published Topic | Description |
| ------------------ | ----------- |
| /traffic_waypoint | contains either RED or -1 for the current image from /image_color topic |


Here's the [Github URL for TF Classifier] which takes you through the complete details of the classifier.
In brief, this is what this module does

* Traffic light detection

    Every autonomous vehicle's main perception comes through the camera feed. A cemera feed is nothing but a series of images. This node continuously processes these camera images through a Traffic light detection module to precisely identify the location of the traffic light in the image. This module is built using on top of the Tensorflow's Object detection model. If there's no traffic light detected in a particular image, nothing is returned.
    
    
* Traffic Light Classification

    Once the precise location of the traffic light is detected, that particular snippet is taken from the image and is sent through a Traffic Light Color Classifier. Humans require color images to identify but the computers(classifiers) can be trained to work on gray images so that computation is faster. The traffic light snippets will be converted to gray scale and hued [Omkar TODO] to finally classify the color of the light
    
    The output of this module will be the upcoming waypoint index whose traffic light is RED. It returns `-1` if there's no traffic light in the image. To ensure the classification is 100% correct, we've implemented a logic to wait for atleast 3 consequent classifications of RED before its published to the `/traffic_waypoints` topic.
    

#### Waypoint Generation & Updation

This node forms a part of the Path Planning module for the vehicle

| Topic Subscription | Description |
| ------------------ | ----------- |
| /base_waypoints    | A map of all the waypoints of the environment |
| /current_pose      | Provides the current stats of the vehicle like current position, linear & angular velocities, etc |
| /traffic_waypoint  | Contains the traffic light classification result from previous node |

| Published Topic | Description |
| ------------------ | ----------- |
| /final_waypoints | Publishes next n waypoints with associated linear and angular velocities |

   This node is responsible to publish the waypoints and associated velocities that the car needs to take. It only publishes so many waypoints that the vehicle needs to care about. For ex, the vechicle need not adjust itself for something that 200m away from its current position. In the implementation, we considered the next 150 waypoints as an optimum number as we felt that this would give enough time for the vehicle to comfortably decelerate and adjust if a RED signal's approaching.

   This node gets the upcoming traffic light information from `/traffic_waypoint` topic. If the upcoming signal is `RED`, then the vehicle needs to decelerate so that it stops at the traffic signal. For this, the waypoints from its current location until the traffic signal are calculated (provided they are within 150 waypoints) and corresponding velocities for each waypoint are associated so that its velocity when it reaches the `RED` traffic signal is 0. 
        Deceleration velocities are calculated using a simple formula `\sqrt{2 * MAX_DECEL * dist}`
        
#### Waypoint follower

This node is responsible to take the input from waypoint generation node and convert them in to `Twist` commands(Linear & Angular accelerations) for the controller module. This node contains software from [Autoware](https://github.com/CPFL/Autoware) and nothing has been customized
        
#### Controls
This node forms the control module for the Vehicle

| Topic Subscription | Description |
| ------------------ | ----------- |
| /twist_cmd         | Provides the Twist commands (linear & angular velocities) for the waypoints |
| /current_velocity  | Provides the current velocity of the vehicle |
| /vehicle/dbw_enabled | Lets us know if the vehicle is controlled by software or manually |

| Published Topic | Description |
| ------------------ | ----------- |
| /vehicle/steering_cmd | Publishes the Steering Command to the Vehicle |
| /vehicle/throttle_cmd | Publishes the Throttle Command to the Vehicle |
| /vehicle/brake_cmd | Publishes the Brake Command to the Vehicle |

   Carla is equipped with a system called *Drive By Wire(DBW)* which is a software-regulated system for controlling the engine, handling, suspension, and other functions of a motor vehicle. As part of this node, Twist commands from the `/twist_cmd` topic will be converted to Steering, Brake & Throttle commands and publish to corresponding topics


* Steering angle
    
    The incoming velocity is passed through a LowPassFilter to remove the high frequency noise. It is then passed through a Yaw Controller to calculate the steering angle.
    
    
* Brake 

    Brake is calculated based on the difference between the current velocity and the target linear velocity, and taking maximum deceleration limit into consideration. For obvious reasons, you don't want to break suddenly and that's why max deceleration limit is defined. The magnitude of braking is calculated in Newton-meter, and it depends on the vehicle mass and wheel radius. The higher are these terms, the more force you need to apply to slow down the vehicle
    
    
* Throttle

    PID Controller is used to determine the amount of throttle required. Based on the difference between the target velocity and current velocity, the error is calculated, and the time is taken into consideration to calculate the linear distance error and applied to a PID controller to determine the final throttle value
    
    
### Execution

Traffic light classification, way point generation are compute intensive, and a GPU is required to run them seemlessly. Udacity provided a GPU enabled simulator which is a Ubuntu Installation with ROS. While most of the code is built and run within the workspace, training the classifier took a lot of time and Udacity's 50 hrs was not enough to successfully test various models. One of our team members had a local GPU on his machine, and we used his machine to train the classifier. Once the classifier is ready, we've plugged it in to the workspace and everything went smooth

### Results

The aim of the project is to successfully run the car within the simulator that follows waypoints, lanes and performs required actions at the traffic signals. After the individual components are integrated within the simulator, we observed tha car was taking the waypoints correctly, classifier is classifying the color of traffic lights correctly and stopping at RED signals, continuing at Green signals. For yellow, we have considered a buffer distance, and based on that, it will either continue if its safe to drive, or stop if the distance is more than a threshold to continue.

After we could succesfully test in the simulator, Udacity provided a ROSBag that contained the real test track in Palo Alto, CA. We've used our model to run in this track, and the car performed correctly wihtout any issues

### Lessons Learned
   One of the main lessons learned during this project is 'working with teams'. The number of ideas that flowed through as we were working in a team are way beyond compared to what would have come when working individually. Each of us had their own personal strenghts w.r.t. the technology, and each of us learnt from others during hte course of hte project

   All through the course, we were concentrating on specific modules of Autonomous Vehicle technologies, but this is the project where it all got together. It is a tremendous learning experience working in the project to completely understand the underlying infrastructure of a Self-driving car



### Installation & Setup Instructions

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
