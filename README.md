# SLAM

## Installation ##

First install depthai-core as it provides the drivers and other C# packages to allow the OS to communicate with the camera. This can be done in the root directory.

If you get an error due to depthaiConfig it is most likely because this library has not been built as a shared library. Try removing the hunter cache ``` rm -r ~/.hunter ``` and trying again with the below commands. If it fails considering using less cores in make (-j2 or -j1). The depthai-core git repository also has new instructions to make it work.

```
cd ~
git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
cd depthai-core
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
```

Next create a project directory using catkin_make and cloning this directory which contains launch files that make use of depthai-ros-examples, rtabmap_ros and move_base.
To create and install:
```
#Create your target catkin workspace
cd ~
mkdir slam_ws/          
# Create a src folder containing this repo
git clone https://github.com/Abi-Humanoid/abi_navigation.git src        
catkin_make 
# Add to path
source devel/setup.bash         
```

The depthai-ros packages can then be installed which acts as a depthai-core wrapper for ROS. This package comes with examples that can easily be used.
Ensure rosdep is installed and updated, the vcstool is also implemented, and libopencv.
If this causes the system to crash then try running with less calls (``` catkin_make -j1 ```)
```
# Install OpenCV libraries
sudo apt install libopencv-dev

# Install rosdep package
sudo apt install python-rosdep (melodic) or sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install the vcstool  
sudo apt install python3-vcstool

# Install depthai-ros
cd <directory_for_workspaces> (slam_ws)
wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos
vcs import src < underlay.repos
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/melodic/setup.bash
# If issue with next step then use sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv as opencv cannot be found on jetson xavier
catkin_make
source devel/setup.bash
```


rtabmap_ros also needs to be installed:
```
sudo apt-get install ros-melodic-rtabmap-ros
```

move_base also needs to be installed:
```
sudo apt-get install ros-melodic-navigation
```

depthimage_to_lasercloud also needs to be installed:
```
sudo apt-get install ros-melodic-depthimage-to-laserscan
```



---
#Troubleshooting
If the camera can not be detected with some X_LINK error then refer to the troubleshootin of luxonis https://docs.luxonis.com/en/latest/pages/troubleshooting/ . It's most likely a problem with the USB rules.



---
#Usage

This package has each component seperated into different launch files, but they could potentially be placed into one file.
To launch the camera and odometry node:
```
roslaunch mapping camera.launc
```
Next begin by making a scan of the room:
```
roslaunch mapping stereo_mapping.launch```
At this point a ros 

