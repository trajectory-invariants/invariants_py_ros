#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug  4 13:26:16 2023

@author: maxim
"""

import rospy
from std_msgs.msg import MultiArrayDimension, Header, Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def convert_nparray_to_Path(np_array):
    # Convert numpy array to ROS Path message
    path_msg = Path()
    path_msg.header.frame_id = 'world'  # Set the reference frame to "world"
    for point in np_array:
        pose = PoseStamped()
        pose.header.frame_id = 'world'  # Set the reference frame to "world" for each pose
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        path_msg.poses.append(pose)
    return path_msg

def convert_nparray_to_Float32MultiArray(np_array):
    # Convert to MultiArray data type for Publisher
    multiarray = Float32MultiArray()
    multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                             np_array.shape[i],
                                             np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)];
    multiarray.data = np_array.reshape([1, -1])[0].tolist();
    
    return multiarray

def pack_data(data):
    # Define the PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Set your desired frame ID

    # Define the fields for the PointCloud2 message
    fields = [PointField('data', 0, PointField.FLOAT32, 1)]

    # Convert the float array to bytes
    data_bytes = struct.pack('f' * len(data), *data)

    # Create the PointCloud2 message
    pc_data = pc2.create_cloud(header, fields, [(point,) for point in data])
    pc_data.data = data_bytes  # Assign the converted data to the message

    return pc_data       