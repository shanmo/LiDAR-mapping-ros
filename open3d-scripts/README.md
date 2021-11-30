# about 

- Use the rosbag (lidar_data.bag) to create a pointcloud map (in .pcd format)
- The bag contents robot pose, IMU data and LiDAR pointcloud data
```
Input:
/velodyne_points [sensor_msgs/PointCloud2]
/mcu/state/imu [sensor_msgs/Imu] (optional)
/tf (optional)
Output:
map.pcd
```

# platform 

- ubuntu 18.04
- ROS melodic 
- python 2.7 

# results 

- [reconstructed point clouds](https://github.com/shanmo/LiDAR-mapping-ros/issues/1)
- [project description](https://github.com/shanmo/LiDAR-mapping-ros/tree/main/open3d-scripts/doc)

# reference 

- http://www.open3d.org/docs/0.10.0/tutorial/Advanced/multiway_registration.html
