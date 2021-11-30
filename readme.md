## about 

this repo uses a rosbag that contains LiDAR and inertial data to produce a point cloud map 

## how to save pcd map 

- validate rosbag 
```
cartographer_rosbag_validate -bag_filename /home/erl/lidar-mapping/lidar_data.bag
```
output 
```
erl@erl:~/lidar-mapping/LiDAR-mapping-ros/catkin_cartographer_ws$ cartographer_rosbag_validate -bag_filename /home/erl/lidar-mapping/lidar_data.bag
W1130 11:08:48.937381  8230 rosbag_validate_main.cc:103] frame_id imu_link_raw time 1609120319553672114: IMU linear acceleration is 2.285 m/s^2, expected is [3, 30] m/s^2. (It should include gravity and be given in m/s^2.) linear_acceleration 00.386264 0.0719486 0-2.25097
W1130 11:08:48.969158  8230 rosbag_validate_main.cc:103] frame_id imu_link_raw time 1609120323174227849: IMU linear acceleration is 1.76098 m/s^2, expected is [3, 30] m/s^2. (It should include gravity and be given in m/s^2.) linear_acceleration -0.889693 -0.752971 0-1.32005
W1130 11:08:48.990995  8230 rosbag_validate_main.cc:103] frame_id imu_link_raw time 1609120325544140121: IMU linear acceleration is 2.0959 m/s^2, expected is [3, 30] m/s^2. (It should include gravity and be given in m/s^2.) linear_acceleration 0.514429 0.940797 -1.80085
W1130 11:08:49.178902  8230 rosbag_validate_main.cc:166] Sensor with frame_id "velodyne" measurements overlap in time. Previous range message, ending at time stamp 637447171456779964, must finish before current range message, which ranges from 637447171456768925 to 637447171457745872
W1130 11:08:49.758163  8230 rosbag_validate_main.cc:203] Sensor with frame_id "velodyne" range measurements have longest overlap of 0.0011039 s
I1130 11:08:49.758292  8230 rosbag_validate_main.cc:399] Time delta histogram for consecutive messages on topic "/mcu/state/imu" (frame_id: "imu_link_raw"):
Count: 11342  Min: 0.000913843  Max: 0.0178464  Mean: 0.00801149
[0.000914, 0.002607)	                    	Count: 2 (0.0176336%)	Total: 2 (0.0176336%)
[0.002607, 0.004300)	                    	Count: 4 (0.0352671%)	Total: 6 (0.0529007%)
[0.004300, 0.005994)	                    	Count: 32 (0.282137%)	Total: 38 (0.335038%)
[0.005994, 0.007687)	                    	Count: 154 (1.35779%)	Total: 192 (1.69282%)
[0.007687, 0.009380)	 ###################	Count: 11045 (97.3814%)	Total: 11237 (99.0742%)
[0.009380, 0.011073)	                    	Count: 92 (0.811144%)	Total: 11329 (99.8854%)
[0.011073, 0.012767)	                    	Count: 10 (0.0881679%)	Total: 11339 (99.9735%)
[0.012767, 0.014460)	                    	Count: 1 (0.00881679%)	Total: 11340 (99.9824%)
[0.014460, 0.016153)	                    	Count: 1 (0.00881679%)	Total: 11341 (99.9912%)
[0.016153, 0.017846]	                    	Count: 1 (0.00881679%)	Total: 11342 (100%)
E1130 11:08:49.758383  8230 rosbag_validate_main.cc:383] Point data (frame_id: "velodyne") has a large gap, largest is 0.105251 s, recommended is [0.0005, 0.05] s with no jitter.
I1130 11:08:49.758492  8230 rosbag_validate_main.cc:399] Time delta histogram for consecutive messages on topic "/velodyne_points" (frame_id: "velodyne"):
Count: 900  Min: 0.0965931  Max: 0.105251  Mean: 0.100916
[0.096593, 0.097459)	                    	Count: 1 (0.111111%)	Total: 1 (0.111111%)
[0.097459, 0.098325)	                    	Count: 0 (0%)	Total: 1 (0.111111%)
[0.098325, 0.099190)	                    	Count: 0 (0%)	Total: 1 (0.111111%)
[0.099190, 0.100056)	                    	Count: 0 (0%)	Total: 1 (0.111111%)
[0.100056, 0.100922)	       #############	Count: 569 (63.2222%)	Total: 570 (63.3333%)
[0.100922, 0.101788)	             #######	Count: 329 (36.5556%)	Total: 899 (99.8889%)
[0.101788, 0.102654)	                    	Count: 0 (0%)	Total: 899 (99.8889%)
[0.102654, 0.103519)	                    	Count: 0 (0%)	Total: 899 (99.8889%)
[0.103519, 0.104385)	                    	Count: 0 (0%)	Total: 899 (99.8889%)
[0.104385, 0.105251]	                    	Count: 1 (0.111111%)	Total: 900 (100%)
```

- from the output above, we obtain 
```
imu topic: /mcu/state/imu
imu frame_id: imu_link_raw
velodyne topic: /velodyne_points 
frame_id: velodyne
```

- define urdf file 

- run launch 
```
roslaunch cartographer_ros lidar_mapping_3d.launch
```

- note, need to run `catkin_make_isolated --install --use-ninja` every time launch file is modified 

- to save the pcd map, follow [this](https://buildmedia.readthedocs.org/media/pdf/google-cartographer-ros/latest/google-cartographer-ros.pdf), when the online map building finishes, in another terminal run 
```
# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0
# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '${HOME}/Downloads/demo.pbstream', include_unfinished_submaps: "true"}"
```

## results 

- the point cloud map in rviz is in https://github.com/shanmo/LiDAR-mapping-ros/issues/3
- [video demo](https://youtu.be/B6nrmXvqAuo)

## reference 

- https://github.com/xiangli0608/cartographer_detailed_comments_ws

