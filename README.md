# multi-laserscan-toolbox-ros2
This ROS2 package provides a lightweight and modular solution for fusing sensor data from an unrestricted amount of sensors' inputs formatted as Laser-Scans, enabling efficient and accurate 2D navigation or mapping while considering 3D obstacles.
Any robot can easily plug and play this package into their current mapping, or navigation, system using several different sensors.
Our method focuses on merging data from multiple sensors, such as LiDARs, cameras, and ultrasonic sensors, into a unified Laser-Scan, which serves as a foundation for faster and more lightweight navigation. 

By fusing sensor data at the Laser-Scan level, our approach enables the use of basic 2D Simultaneous Localization And Mapping (SLAM) algorithms for mapping tasks, or any others Laser-Scan based features, while still benefiting from the rich information provided by multimodal 3D inputs.
This results in a more computationally efficient solution compared to traditional 3D methods that rely on depth points or full multimodal SLAM systems. 
It does not aim to replace existing methods for 2D mapping and navigation but rather seeks to enhance them by reducing their computational load and integrating data from multiple 3D and 2D sensors.

<table align="center">
  <tr>
    <td align="center">
      <a href="materials/A Lightweight Approach to Efficient Multimodal 2D Navigation and Mapping Unified LaserScans as an Alternative to 3D Methods_uncompressed.mp4">Related Video</a>
    </td>
  </tr>
  <tr>
    <td>
      <video width="430" height="240" controls>
        <source src="materials/A Lightweight Approach to Efficient Multimodal 2D Navigation and Mapping Unified LaserScans as an Alternative to 3D Methods_uncompressed.mp4" type="video/mp4">
      </video>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="materials/A_Lightweight_Approach_to_Efficient_Multimodal_2D_Navigation_and_Mapping_Unified_LaserScans_as_an_Alternative_to_3D_Methods.pdf">Related Paper</a>
    </td>
  </tr>
</table>

## Dependencies
* ROS2 (Tested on Humble): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

The following ROS2 packages are used and should be available once ROS2 is installed:
* ament_cmake
* rclcpp
* sensor_msgs
* tf2
* tf2_ros
* tf2_geometry_msgs
* rosgraph_msgs
* nav_msgs

If needed, install the missing ones:
```
sudo apt-get install ros-<ros_version>-<PACKAGE_name>
```

## Installation

```
cd <your_ros2_workspace>/src/
git clone https://github.com/Noceo200/multi-laserscan-toolbox-ros2.git
cd ..
colcon build
```

## API

### Subscribed topics

| Topic  | Type | Description | 
|-----|----|----|
| /scan1  | `sensor_msgs/LaserScan` | The specified Laser-Scan input from your sensor 1 | 
| /scan2  | `sensor_msgs/LaserScan` | The specified Laser-Scan input from your sensor 2 | 
| /scan_i...  | `sensor_msgs/LaserScan` | An unrestricted amount of inputs can be specified | 
| /odom  | `nav_msgs/Odometry` | The odometry of your robot is used to ensure a consistent fusion | 
| **tf** | N/A | A valid transform from your configured frames and those attached to the Laser-Scan messages inputs |

### Published topics

| Topic  | Type | Description | 
|-----|----|----|
| /scan  | `sensor_msgs/LaserScan` | Output merged Unified Laser-Scan at a virtual specified frame | 

## Configuration

### Merger Params

`mode` - "mapping" or "localization" mode for performance optimizations in the Ceres problem creation

`mode` - "mapping" or "localization" mode for performance optimizations in the Ceres problem creation

### Inputs Params

Those configurations can be specified for each inputs.

`odom_frame` - Odometry frame

`map_frame` - Map frame

`base_frame` - Base frame

## Launch
Two parameters can be specified when launching the package.

`use_sim_time` - If True, the package will subscribed to `/clock` and use this time to synchronise and publish the Laser-Scan messages. Otherwise, the system's clock will be used.

`params_file` - YAML file tu consider for the configurations.

To launch with the default configurations in "config/laserscan_toolbox_params.yaml":
```
ros2 launch multi-laserscan-toolbox-ros2 laserscan_toolbox.launch.py use_sim_time:=<true/false>
```

To launch with your own configurations:
```
ros2 launch multi-laserscan-toolbox-ros2 laserscan_toolbox.launch.py params_file:=<your_yaml_file_path> use_sim_time:=<true/false>
```
