# multi-laserscan-toolbox-ros2
The package 'laser_scan_merger' allow to merge 2 LaserScan messages in one. 

Transformations theory:
http://miageprojet2.unice.fr/@api/deki/files/3019/=1_-_TransformationsGeometriques.pdf

## To-DO
- new name: Multi_laserscan_toolbox
- Merge list of laser scans + output list of wanted virtual lidars (Better if on robot that are lidars, to avoid fake informations if placed somewhere where the closest points are not detected)
- Possibility to do multi mapping, (use slam_toolbox, merge maps), or created multi lidars SLAM. (Autre Git?)

### Limitations
Some informations are lost when converting multiple LaserScans to one.

## Dependencies
* ros2 humble (not tested on other versions)

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
