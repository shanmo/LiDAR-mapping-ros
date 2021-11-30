#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import open3d as o3d

if __name__ == "__main__": 

    pcd_file = "/home/erl/lidar-mapping/LiDAR-mapping-ros/data/demo.pcd"
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(pcd_file)
    o3d.visualization.draw_geometries([pcd])
