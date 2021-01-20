# siec_navigation
This repository contains nodes and packages wrote and used for navigation. 

## install 
First go to your catkin workspace source folder and clone the repository.
```bash
cd ~/catkin_ws/src
git clone https://github.com/siec2020/siec_navigation.git
```
To use the simulator, you would also need an old version of the autoware messages from this repo: (otherwise you would get a checksum error with the newest version as the simulator did not update this message definition) 
```bash
git clone https://github.com/siec2020/siec_navigation.git
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

## demo

In this part, we will launch a navigation demo working with LGSVL simulator and the hmi. 

This repo "repo_lgsvl_siec" will explain to you how to install the LGSVL simulator and the change you need to do to make it properly working with ROS (specially for the car to go in reverse).

This repo https://github.com/siec2020/hmi.git will explain to you how to install the IHM (apache server and javascript and php code) on your machine. --note-- The version of the HMI in this repo is designed to work with the real car, to use the simulator you need to clone the following branch (hmi_simu_lgsvl) of this repo in your www folder instead of the master branch.

```bash
git clone -b hmi_simu_lgsvl https://github.com/siec2020/hmi.git
```

The next procedure assume that the LGSVL simulator and the HMI has already been installed in a proper way. 


