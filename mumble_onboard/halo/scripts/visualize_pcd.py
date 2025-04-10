#!/usr/bin/env python3
import open3d as o3d
import argparse

import open3d as o3d
import os
import numpy as np

# add -s to save camera view.
# camera_view.json should be changed to pcd_file_camera_view.json


def get_camera_config_file_path(file_path):
    file_name = os.path.splitext(file_path)[0].split("/")[-1]
    return f"/tmp/{file_name}_camera_view.json"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize PCD file")
    parser.add_argument("--pcd_file", "-p", type=str, help="Path to the PCD file")
    parser.add_argument(
        "--skip_saving_view",
        "-s",
        action="store_true",
        help="Save the perspective config to X_camera_view.json file",
    )
    args = parser.parse_args()
    # Load point cloud
    pcd = o3d.io.read_point_cloud(args.pcd_file)

    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Interactive View")
    vis.add_geometry(pcd)

    camera_config_path = get_camera_config_file_path(args.pcd_file)
    if os.path.exists(camera_config_path):
        print(f"Loading camera view from {camera_config_path}")
        cam_params = o3d.io.read_pinhole_camera_parameters(camera_config_path)
        vis.get_view_control().convert_from_pinhole_camera_parameters(cam_params)
    else:
        print(
            f"No saved camera view found at {camera_config_path}. Starting with default view."
        )

    if args.skip_saving_view:
        print(
            "\033[91mCamera view will not be saved. Remove -s to save the current view.\033[0m"
        )
    else:
        print(f"\033[96mCamera view will be saved to {camera_config_path}.\033[0m")

    # Run GUI
    vis.run()

    # Save the camera view after interaction
    if not args.skip_saving_view:
        current_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
        o3d.io.write_pinhole_camera_parameters(camera_config_path, current_params)
        print(f"Camera view saved to {camera_config_path}")

    vis.destroy_window()
