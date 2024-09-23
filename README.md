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
| /scani ...  | `sensor_msgs/LaserScan` | An unrestricted amount of inputs can be specified | 
| /odom  | `nav_msgs/Odometry` | The odometry of your robot is used to ensure a consistent fusion | 
| **tf** | N/A | A valid transform from your configured frames and those attached to the Laser-Scan messages inputs |

### Published topics

| Topic  | Type | Description | 
|-----|----|----|
| /scan  | `sensor_msgs/LaserScan` | Output merged Unified Laser-Scan at a virtual specified frame | 

## Configuration

### Common Params

`rate` - The frequency at which the node operates, in Hz. (Might be smaller, depending on the specified parameter `extrapolate_scan`)

`topic_out` - The topic name for the output scan.

`new_frame` - The frame in which the new scan will be published.

`odom_topic` - The topic name for odometry data, used by the persistence feature and to synchronize scans.

`odom_delay_limit` - The time in seconds for which odometry data is kept in memory. A value of 0.0 means only the last received odometry is kept.

`extrapolate_odometry` - A boolean flag that defines if the program should extrapolate the odometry when the wanted stamp of a LaserScan is after the newest received odometry.

### Debugging

`debug` - A boolean flag to enable or disable debugging.

`debug_file_path` - The file path where debug information will be saved.

`show_ranges` - A boolean flag to enable or disable the display of ranges of the output scan.

`show_odometry_detail` - A boolean flag to enable or disable the display of detailed odometry information.

### Output Settings

`angle_min` - The minimum angle of the scan in radians (specified in the `new_frame` frame).

`angle_max` - The maximum angle of the scan in radians (specified in the `new_frame` frame).

`angle_increment` - The angular increment between each measurement in radians.

`range_min` - The minimum range of the scan in meters (specified in the `new_frame` frame).

`range_max` - The maximum range of the scan in meters (specified in the `new_frame` frame).

`extrapolate_scan` - A boolean flag that, if true, allows the node to respect the `rate` by updating the scan according to the new received odometry even if the sensors didn't update new data. If false, the merged scan will only be updated and published when new data from one of the sensors are received.

### Advanced Output Settings

`auto_set` - A boolean flag that, if true, will automatically set the following settings using the first source.

`time_increment` - The time increment in seconds.

`scan_time` - The scan time in seconds.

### Sources

These configurations can be specified for each input source.

`sources` - A list of source identifiers (e.g., `s1`, `s2`, `s3`, `s4`).

#### Source-Specific Params (e.g., `s1`, `s2`, `s3`, `s4`)

`topic` - The topic name for the input 2D Laser-Scan.

`timeout` - The timeout in seconds for the input scan. A value of 0.0 means all values are authorized.

`start_angle` - The starting angle of the scan in radians (specified in the source frame).

`end_angle` - The ending angle of the scan in radians (specified in the source frame).

`scan_angle_offset` - The angle offset of the scan in radians.

`range_min` - The minimum range of the scan in meters (specified in the source frame).

`range_max` - The maximum range of the scan in meters (specified in the source frame).

`persistence` - A boolean flag for the persistence feature (currently under development).

`timeout_persistence` - The timeout for points to keep in the persistent scan in seconds. A value of 0.0 means all values are kept (currently under development).


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
