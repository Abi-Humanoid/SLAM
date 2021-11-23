# SLAM

## Installation ##

First install depthai-core as it provides the drivers and other C# packages to allow the OS to communicate with the camera. This can be done in the root directory.

```
cd ~
git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
cd depthai-core
mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
```

Next create a project directory using catkin_make and cloning this directory which contains the orb_slam2 and rtabmap_ros packages for the OAK-D camera.
To create and install:
```
#Create your target catkin workspace
cd ~
mkdir slam_ws/          
# Create a src folder containing this repo
git clone https://github.com/Abi-Humanoid/SLAM.git src        
catkin_make 
# Add to path
source devel/setup.bash         
```

The depthai-ros packages can then be installed which acts as a depthai-core wrapper for ROS. This package comes with examples that can easily be used.
Ensure rosdep is installed and updated, the vcstool is also implemented, and libopencv.
```
# Install OpenCV libraries
sudo apt install libopencv-dev

# Install rosdep package
sudo apt install python-rosdep2(melodic) or sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install the vcstool  
sudo apt install python3-vcstool

# Install depthai-ros
cd <directory_for_workspaces> (slam_ws)
wget https://raw.githubusercontent.com/luxonis/depthai-ros/noetic-devel/underlay.repos
vcs import src < underlay.repos
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```


Next orb_slam2_ros needs to be installed:
```
cd src
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
cd ..
catkin_make
source devel/setup.bash
```

rtabmap_ros also needs to be installed:
```
sudo apt-get install ros-melodic-rtabmap-ros
```

Now the depth_image_proc package needs to be installed/updated:
```
sudo apt-get install ros-melodic-depth-image-poc
```



---
The ```orb2_slam/stereo_orb2_slam.launch``` is the only launch file that currently works.

The output of this is a PointCloud2 in the slam/map_points topic.
When this topic is viewed in rviz the features of the room can be viewed.

```orb2_slam/rgbd_orb2_slam.launch``` does not work due to the ```depth_image_proc/register``` topic not working properly with the OAK-D's depth image.

Neither of the rtabmap packages have been successfully implemented for reasons outlined in https://github.com/introlab/rtabmap/issues/742 (Updates on this section might suggest rtabmap is possible) 
