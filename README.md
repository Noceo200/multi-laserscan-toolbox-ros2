# multi-laserscan-toolbox-ros2
This ROS2 package provides a lightweight and modular solution for fusing sensor data from an unrestricted amount of sensors' inputs formatted as Laser-Scans, enabling efficient and accurate 2D navigation or mapping while considering 3D obstacles.
Any robot can easily plug and play this package into their current mapping, or navigation, system using several different sensors.
Our method focuses on merging data from multiple sensors, such as LiDARs, cameras, and ultrasonic sensors, into a unified Laser-Scan, which serves as a foundation for faster and more lightweight navigation. 

By fusing sensor data at the Laser-Scan level, our approach enables the use of basic 2D Simultaneous Localization And Mapping (SLAM) algorithms for mapping tasks, or any others Laser-Scan based features, while still benefiting from the rich information provided by multimodal 3D inputs.
This results in a more computationally efficient solution compared to traditional 3D methods that rely on depth points or full multimodal SLAM systems. 
It does not aim to replace existing methods for 2D mapping and navigation but rather seeks to enhance them by reducing their computational load and integrating data from multiple 3D and 2D sensors.

<div style="text-align: center;">
  <video width="430" height="240" controls>
    <source src="materials/A Lightweight Approach to Efficient Multimodal 2D Navigation and Mapping Unified LaserScans as an Alternative to 3D Methods_uncompressed.mp4" type="video/mp4">
  </video>
  <br>
  <a href="materials/A_Lightweight_Approach_to_Efficient_Multimodal_2D_Navigation_and_Mapping_Unified_LaserScans_as_an_Alternative_to_3D_Methods.pdf">Related Paper</a>
</div>

## Dependencies
* ROS2 (Tested on Humble): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### Packages needed

The following ros2 packages should be available:
* ament_cmake
* rclcpp
* sensor_msgs
* tf2
* tf2_ros
* tf2_geometry_msgs

If needed, install the missing ones:
```
sudo apt-get install ros-<ros_version>-<PACKAGE_name>
```

## Installation

Install:
'laser_scan_merger' is a package only tested on <br>ros2 humble</br>.
```
cd <your_ros2_workspace>/src/
git clone https://github.com/Noceo200/laser_scan_merger.git
cd ..
colcon build
```

## Edit behavior of the scan merger

Open 'laser_scan_merger/config/params.yaml':
```
laser_scan_merger_node:
  ros__parameters:
    lidar1_start_angle : 4.71238898 #270 deg
    lidar1_end_angle : 3.141592654 #180 deg
    lidar2_start_angle : 1.570796327 #90 deg
    lidar2_end_angle : 6.283185307 #360 deg
    topic_lid1 : "lidar1_scan" #Topic to subscribe for Lidar1's LaserScan
    topic_lid2 : "lidar2_scan" #Topic to subscribe for Lidar2's LaserScan
    topic_out : "scan" #Output Topic
    new_frame : "base2_link" #Frame to use for the new virtual Lidar created
```

Rebuild:
```
colcon build
```

#### Launch Scan merger
To launch with yaml parameters:
```
ros2 launch laser_scan_merger launch.py
```

To launch with default parameters:
```
ros2 run laser_scan_merger laser_scan_merger_node
```
