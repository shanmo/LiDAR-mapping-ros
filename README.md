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

# reference 

- http://www.open3d.org/docs/0.10.0/tutorial/Advanced/multiway_registration.html