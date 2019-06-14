# Integration of a RGB-D camera in a opration lamp

Author: Junsheng Ding

Project supervisor: [Christian Marzi]

This project is developed with ORB-Slam2 and Intel depth camera D415. The ros_rgbd.cc from original ORB-Slam2 has been mordified and some more features are added:
  - publish RGBD point cloud in Rviz 
  - calibrate camera pose with a chessboard and ART system in OP-Labor
  - publish camera pose in OP-Labor as tf in tf_tree 

# 1. Prerequisites
### 1.1 ROS
This project is tested with ROS Melodic.
http://wiki.ros.org/melodic

Configure the ROS network in the computer which runs ORB-SLAM2 and the Lamp-Server which publishes the camera nodes, to enable communication with ROS Master centralservices.

### 1.2 Drivers for D415
(If you don't use D415 in this project，skip to 1.3.)

Install Intel® RealSense™ SDK 2.0 for RGBD video streaming from https://github.com/IntelRealSense/librealsense. 
Then install ROS Wrapper for Intel® RealSense™ Devices from https://github.com/IntelRealSense/realsense-ros.

Test with the following command and check if topics camera/color/image_raw and camera/depth/image_rect_raw are published.
```sh
$ roslaunch realsense2_camera rs_camera.launch
```
### 1.3 ORB-Slam 2
Install ORB-Slam2 from https://github.com/raulmur/ORB_SLAM2 and install relevant prerequisites.

### 1.4 ART-System

ART-System in OP-Labor is used in this project to estimate the absolute position of D415 in the laboratory. Make sure that the chessboard is detected by ART-System before it appears in the perspective of D415.

# 2 Installation
Replace ORB-SLAM2/Examples/ROS/ORB-SLAM2/src/ros_ros_rgbd.cc with ros_ros_rgbd.cc in this project and recompile. (If other RGBD camera than D415 is used, line 79 and 80 of ros_ros_rgbd.cc should be modified with  cooresponding camera topics.)

Then add camera parameters to setting file
ORB-SLAM2/Examples/ROS/ORB-SLAM2/Asus.yaml and run main programm.

```sh
$ rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

If this programm runs successfully, the camera pose should be published in tf_tree and can be visualized with Rviz.
![Image text](https://github.com/Utrrgkit/Integration-einer-RGB-D-Kamera-in-eine-OP-Lampe/blob/master/Camera%20pose%20and%20point%20cloud%20in%20RVIZ.png)
If the camera is calibrated successfuly with ART-System, the tf_tree should also link with ART:
![Image text](https://github.com/Utrrgkit/Integration-einer-RGB-D-Kamera-in-eine-OP-Lampe/blob/master/tf_tree.png)



   [Christian Marzi]: <https://www.ipr.kit.edu/mitarbeiter_2640.php>

