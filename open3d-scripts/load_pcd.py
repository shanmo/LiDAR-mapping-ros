#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import ros_numpy  # apt install ros-melodic-ros-numpy
import rosbag
import sensor_msgs
import copy
import open3d as o3d

# ref https://stackoverflow.com/a/67296769
def convert_pc_msg_to_np(pc_msg):
    # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
    pc_msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    return pc_np


def load_pcd_all(bag_file, pcd_topic, voxel_size): 
    pcd_all = []
    # To run it offline on a rosbag use:
    for topic, msg, t in rosbag.Bag(bag_file).read_messages():
        if topic == pcd_topic:
            pc_np = convert_pc_msg_to_np(msg)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc_np)
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            pcd_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            pcd_all.append(pcd_down)
    return pcd_all

if __name__ == "__main__": 

    bag_file = "/home/erl/lidar-slam/lidar_data.bag"
    pcd_topic = "/velodyne_points"
    voxel_size = 0.02

    pcd_all = load_pcd_all(bag_file, pcd_topic, voxel_size)
    print("load pcd from {} frames".format(len(pcd_all)))

    o3d.visualization.draw_geometries(pcd_all)
