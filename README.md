# SLAM

For the OAK-D camera to work a number of packages are required. 

These can be done by following the steps outlined in https://github.com/luxonis/depthai-ros
This installs the depthai-core library, as well as the ros package for depthai.

Next orb_slam2_ros needs to be installed:
```
sudo apt-get install ros-melodic-orb-slam2-ros
```
rtabmap_ros also needs to be installed:

```
sudo apt-get install ros-melodic-rtabmap-ros
```

---
The ```orb2_slam/stereo_orb2_slam.launch``` is the only launch file that currently works.

The output of this is a PointCloud2 in the slam/map_points topic.
When this topic is viewed in rviz the features of the room can be viewed.

```orb2_slam/rgbd_orb2_slam.launch``` does not work due to the ```depth_image_proc/register``` topic not working properly with the OAK-D's depth image.

Neither of the rtabmap packages have been successfully implemented.
