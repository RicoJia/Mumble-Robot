#!/usr/bin/env python3
import open3d as o3d
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize PCD file")
    parser.add_argument("--pcd_file", "-p", type=str, help="Path to the PCD file")
    args = parser.parse_args()
    # Load the PCD file
    pcd = o3d.io.read_point_cloud(args.pcd_file)
    o3d.visualization.draw_geometries([pcd])