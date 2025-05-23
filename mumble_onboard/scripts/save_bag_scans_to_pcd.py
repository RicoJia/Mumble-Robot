#!/usr/bin/env python3
"""
Save each PointCloud2 scan from a ROS2 bag to individual .pcd files,
filtering points by a configurable range.

Usage:
    ros2 run mumble_onboard save_bag_scans_to_pcd.py --bag-path /home/mumble_robot/bags/mojave_apt1/ --max-range 4.5 \
        [--output-dir /tmp/scans] \
        [--min-range 0.3] [--max-range 40.0]

Dependencies:
    pip install open3d numpy
    (and ROS2 packages: rclpy, rosbag2_py, sensor_msgs)
"""

import os
import sys
import argparse

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message

import rosbag2_py
from sensor_msgs.msg import PointCloud2, PointField

import open3d as o3d
import numpy as np


def pointcloud2_to_xyz_array(cloud: PointCloud2) -> np.ndarray:
    """
    Convert PointCloud2 to Nx3 numpy array of XYZ using manual parsing.

    This builds a structured numpy dtype from the PointCloud2.fields offsets
    and point_step, then extracts x,y,z columns and filters out NaNs.
    """
    # Build numpy dtype matching the PointCloud2 layout
    names = [f.name for f in cloud.fields]
    formats = [np.float32 if f.datatype == PointField.FLOAT32 else np.uint8
               for f in cloud.fields]
    offsets = [f.offset for f in cloud.fields]
    dtype = np.dtype({'names': names,
                      'formats': formats,
                      'offsets': offsets,
                      'itemsize': cloud.point_step})
    # Interpret raw buffer
    arr = np.frombuffer(cloud.data, dtype=dtype)
    # Stack XYZ
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
    # Remove NaNs
    mask = ~np.isnan(xyz).any(axis=1)
    return xyz[mask]


def main(argv=None):
    rclpy.init(args=argv)
    node = Node('save_bag_scans')

    parser = argparse.ArgumentParser(
        prog='ros2 run mumble_onboard save_bag_scans',
        description='Export PointCloud2 messages to .pcd with filtering')
    parser.add_argument('-b', '--bag-path', required=True,
                        help='Path to ROS2 bag directory')
    parser.add_argument('-o', '--output-dir', default='/tmp/scans',
                        help='Directory for output .pcd files')
    parser.add_argument('--min-range', type=float, default=0.0,
                        help='Keep points >= this distance (m)')
    parser.add_argument('--max-range', type=float, default=1e6,
                        help='Keep points <= this distance (m)')
    parser.add_argument('--voxel-size', type=float, default=0.0,
                        help='Voxel grid leaf size (m); 0 = off')
    args = parser.parse_args(argv[1:])

    bag_name = os.path.basename(os.path.normpath(args.bag_path))

    # Open the bag
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py._storage.StorageOptions(uri=args.bag_path, storage_id='sqlite3'),
        rosbag2_py._storage.ConverterOptions('', '')
    )

    # Find first PointCloud2 topic
    topics = reader.get_all_topics_and_types()
    pc2_topics = [t for t in topics if t.type.endswith('PointCloud2')]
    if not pc2_topics:
        node.get_logger().error('No PointCloud2 topics found')
        sys.exit(1)
    topic = pc2_topics[0].name
    node.get_logger().info(f'Using topic: {topic}')
    reader.set_filter(rosbag2_py._storage.StorageFilter(topics=[topic]))

    os.makedirs(args.output_dir, exist_ok=True)
    idx = 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        cloud_msg = deserialize_message(raw, PointCloud2)

        # Parse XYZ manually
        xyz = pointcloud2_to_xyz_array(cloud_msg)

        # Range filtering
        if args.min_range > 0.0 or args.max_range < 1e6:
            d = np.linalg.norm(xyz, axis=1)
            mask = (d >= args.min_range) & (d <= args.max_range)
            xyz = xyz[mask]

        # Convert to Open3D and down-sample
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        if args.voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(args.voxel_size)

        # Save .pcd
        fn = os.path.join(args.output_dir, f"{bag_name}_{idx}.pcd")
        o3d.io.write_point_cloud(fn, pcd, write_ascii=False)
        node.get_logger().info(f'Saved {fn} ({len(pcd.points)} pts)')
        idx += 1

    node.get_logger().info(f'Finished export: {idx} frames → {args.output_dir}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
