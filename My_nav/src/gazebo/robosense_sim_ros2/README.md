# robosense_sim_ros2

Gazebo Classic simulation plugin for the RoboSense Airy Lite integration in
this ROS 2 Humble workspace. The implementation is derived from the retained
`livox_laser_simulation_RO2` package, but publishes only a standard
`sensor_msgs/msg/PointCloud2` message compatible with the local `rslidar_sdk`
XYZIRT layout.

Default interface:

- topic: `/rslidar_points`
- frame: `rslidar`
- fields: `x`, `y`, `z`, `intensity`, `ring`, `timestamp`
- rate: 10 Hz
- rings: 24

Usage from xacro:

```xml
<xacro:include filename="$(find robosense_sim_ros2)/urdf/airy_lite.xacro" />
<xacro:airy_lite name="rslidar" parent="base_link" topic="/rslidar_points">
  <origin xyz="0.12 0 0.175" rpy="0 0 0" />
</xacro:airy_lite>
```

The scan CSV is an approximation derived from the former Livox simulation
pattern. It provides software-interface compatibility, not a calibrated
optical reproduction of the physical Airy Lite.
