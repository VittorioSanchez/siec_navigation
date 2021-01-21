# siec_navigation
This repository contains nodes and packages wrote and used for navigation. 

## Install 
First go to your catkin workspace source folder and clone the repository.
```bash
cd ~/catkin_ws/src
git clone https://github.com/siec2020/siec_navigation.git
```
To use the simulator, you would also need an old version of the autoware messages from this repo: https://github.com/bbrito/autoware_msgs.git(otherwise you would get a checksum error with the newest version as the simulator did not update this message definition) 
```bash
git clone https://github.com/bbrito/autoware_msgs.git
```
Then go back to your catkin_ws and build your workspace
```bash
cd ..
catkin_make
```
Here you may get error cause these package need other dependencies. To correct this, just install the missing dependencie and retry building your workspace. Do that last step  until you manage to build your catkin_ws (all dependencies installed). To install dependencies you can either do it from binaries if the package is available: 
```bash
sudo apt-get instal ros-melodic-YOUR-MISSING-PACKAGE-HERE
```
or from github by cloning it in your catkin_ws/src folder: 
```bash
cd ~/catkin_ws/src
git clone https://github.com/your_missing_repo_here
```
If you are familiar with ROS, you can also use rosdep. 

Here is a list of the package that need to be cloned in to your catkin_ws/src (those not available through apt-get):
```bash
//public
async_web_server_cpp
web_video_server
autoware_msgs
rviz_camera_stream  
gps_goal
//siec2020 work 
robot_siec_setup_tf
follow_waypoints_SIEC         
SIEC_control_simu
SIEC_gps_simu
SIEC_navigation_simu
```
Make sure to source your catkin_ws once you are done with the installation to be sure your packages are visible:
```bash
source ~/catkin_ws/devel/setup.bash
```

## Demo

### Prerequisites

In this part, we will launch a navigation demo working with LGSVL simulator and the hmi. 

This repo "repo_lgsvl_siec" will explain to you how to install the LGSVL simulator and the change you need to do to make it properly working with ROS (specially for the car to go in reverse).

This repo https://github.com/siec2020/hmi.git will explain to you how to install the IHM (apache server and javascript and php code) on your machine. --note-- The version of the HMI in this repo is designed to work with the real car, to use the simulator you need to clone the following branch (hmi_simu_lgsvl) of this repo in your www folder instead of the master branch.

```bash
git clone -b hmi_simu_lgsvl https://github.com/siec2020/hmi.git
```

The next procedure assume that the LGSVL simulator and the HMI has already been installed in a proper way. 

### Launch the demo 

#### LGSVL side 

Create a new simulation with the siec vehicle and its JSON config file. Choose the "campus_flat" map. Type your socket adress in the bridge windows (adress of your ROS machines in your local network followed by ':9090' (the default port for rosbridge-websocket). Then run the simulation.

#### ROS side

For ROS, the procedure is made fairly easy using launch files. You only need to launch one launch file in a single command and it will take care of launching all the nodes needed for the demo: RVIZ preconfigured, map_server with a good map, static and dynamic tf broadcaster, rosbridge_websocket (to communicate with the HMI and the simulator through TCP websocket protocol on port 9090), wp_builder_gps, wp_follow_list, move_base(navigation) and a few others.... To launch all those nodes, simply use this command:

```bash
roslaunch SIEC_navigation SIEC_demo_simu.launch
```
If error occur at this point it might be due to some packages still missing, so be sure to install them using one of the methods already explained.

### Understand the interface

#### RVIZ 
If every things works fine, you should see the RVIZ windows popup with a partial map (occupancy grid) of the INSA campus, the spawn point of the car in the simulator being the intersection GEI-GM-STPI. The red dots represents the simulated lidar from LGSVL. This topic (laser scan) can be used to create a 2d map of the campus using gmmaping for instance (this is how the partial map displayed on RVIZ had been created). The rectangle roughtly represent the footprint of the car. When waypoints are set using either RVIZ or the HMI, they will be displayed on RVIZ as red arrows once the "follow" button will be pressed on the HMI. Once the follow button is pressed, it will start the navigation. The car will start to move and the trajectory that the navigation node has computed and will try to follow will be displayed in green. 

#### HMI
At this point you should be able to connect on the HMI to control and observe the behavior of the car, this is how a user will interact with the autonomous car in real life (not with the simulator but with the actual siec car). To connect to the HMI, open your browser and type your loopback adress: 127.0.0.1. You should be greated by the HMI, if not please be sure you have done the HMI installation properly following this procedure : https://github.com/siec2020/hmi.git. From the HMI you can send waypoints to the navigation node using the googlemaps API. The edit button will allow you to add waypoints through clicking on the desired location on the map, the "remove last wp" and "remove wp" will allow you to correct mistakes on the trajectory or delet it entirely and finally the "follow" button will start the navigation. If a navigation is running, clicking on an other button will stop it. A feed back of RVIZ is also displayed on the HMI. The expected behavior is shown in the gif below:  

![](demo_nav_hmi.gif)


 



