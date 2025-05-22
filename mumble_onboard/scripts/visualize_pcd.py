#!/usr/bin/env python3

"""
PCD Viewer: visualize a single .pcd file with perspective camera angles

Usage:
    ros2 run mumble_onboard visualize_pcd.py -p /tmp/edge_points0.pcd

"""

import time
import argparse, os
import numpy as np
import open3d as o3d
from matplotlib.cm import get_cmap

# add -s to save camera view.
# camera_view.json should be changed to pcd_file_camera_view.json

def get_camera_config_file_path(file_path):
    file_name = os.path.splitext(file_path)[0].split("/")[-1]
    return f"/tmp/{file_name}_camera_view.json"

def colorize_by_distance(
        pcd,
        cmap_name="turbo",      # very vivid rainbow
        clip=(5, 95),           # throw away lowest 5 % and highest 5 %
        gamma=0.8,               # <1 = boost mid-tones, >1 = compress
        cmap_range=(0.2, 1.0)   # ← skip the first 20% of the colormap
    ):
    pts   = np.asarray(pcd.points)
    dists = np.linalg.norm(pts, axis=1)

    lo, hi = np.percentile(dists, clip)
    d_clip = np.clip(dists, lo, hi)

    dn = (d_clip - lo) / (hi - lo + 1e-8)
    dn = dn ** gamma

    # remap into [cmap_lo, cmap_hi] instead of [0,1]
    cmap_lo, cmap_hi = cmap_range
    dn = cmap_lo + (cmap_hi - cmap_lo) * dn

    cmap   = get_cmap(cmap_name)
    colors = cmap(dn)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)



def visualize(pcd_path, save_view):
    # load
    pcd = o3d.io.read_point_cloud(pcd_path)
    # ---------- usage inside your visualiser ----------
    colorize_by_distance(pcd, cmap_name="turbo", clip=(5,95), gamma=0.8)

    # setup window
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="PCD Viewer",
        width=1280, height=720,
        left=50, top=50
    )
    vis.add_geometry(pcd)
    # add axes at the origin
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

    ctr = vis.get_view_control()
    # camera‐config file next to your PCD
    cam_cfg = os.path.splitext(pcd_path)[0] + "_cam.json"
    if os.path.exists(cam_cfg):
        params = o3d.io.read_pinhole_camera_parameters(cam_cfg)
        ctr.convert_from_pinhole_camera_parameters(params)

    # tweak render options
    ro = vis.get_render_option()
    ro.point_size = 1.0    # default is ~2.0; lower = smaller points
    ro.background_color = np.asarray([0.05, 0.05, 0.05])
    ro.show_coordinate_frame = True

    try:
        while True:
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\033[91mKeyboardInterrupt caught, shutting down...\033[0m")
        # optionally save the final camera
        if save_view:
            params = ctr.convert_to_pinhole_camera_parameters()
            o3d.io.write_pinhole_camera_parameters(cam_cfg, params)
            print(f"Saved camera parameters to {cam_cfg}")

        vis.destroy_window()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Visualize PCD file")
    p.add_argument("-p", "--pcd",       required=True, help="input .pcd file")
    p.add_argument("-s", "--save-view", action="store_true",
                   help="save final camera to PCDNAME_cam.json")
    args = p.parse_args()
    visualize(args.pcd, args.save_view)




