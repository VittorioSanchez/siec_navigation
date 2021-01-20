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




