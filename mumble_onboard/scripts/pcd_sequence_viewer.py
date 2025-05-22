#!/usr/bin/env python3
"""
PCD Sequence Viewer with colorization and arrow-key stepping. Saves a single
camera-parameters JSON on exit, named after the input path (folder or file).
ESC key closes and triggers JSON save.

Usage:
    python3 pcd_sequence_viewer.py <path_to_pcd_or_folder> \
        [--min-range 0.3] [--max-range 40.0] [--clip 5 95] [--gamma 0.8] [--cmap turbo]

Features:
- Lists .pcd files in a folder or opens a single file
- Numerically sorts frames by trailing index
- Distance-based filtering and colorization
- Arrow keys (← / →) to step frames with wrap-around
- ESC to exit and save camera parameters to a single JSON
"""
import os
import sys
import re
import argparse
import numpy as np
import open3d as o3d
from matplotlib.cm import get_cmap

# Colorization utility
def colorize_by_distance(pcd, cmap_name="turbo", clip=(5, 95), gamma=0.8, cmap_range=(0.2, 1.0)):
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return
    dists = np.linalg.norm(pts, axis=1)
    lo, hi = np.percentile(dists, clip)
    d_clip = np.clip(dists, lo, hi)
    dn = ((d_clip - lo) / (hi - lo + 1e-8)) ** gamma
    cmap_lo, cmap_hi = cmap_range
    dn = cmap_lo + (cmap_hi - cmap_lo) * dn
    colors = get_cmap(cmap_name)(dn)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)

# List .pcd files, numerically sorted
def list_pcd_files(path):
    if os.path.isdir(path):
        files = [os.path.join(path, f) for f in os.listdir(path) if f.lower().endswith('.pcd')]
        def idx_key(f):
            m = re.search(r'_(\d+)\.pcd$', f)
            return int(m.group(1)) if m else -1
        return sorted(files, key=idx_key)
    elif os.path.isfile(path) and path.lower().endswith('.pcd'):
        return [path]
    else:
        print(f"No .pcd files found at {path}")
        sys.exit(1)

class SequenceViewer:
    def __init__(self, files, args, session_json):
        self.files = files
        self.args = args
        self.session_json = session_json
        self.idx = 0
        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.ctr = None

    def load_frame(self):
        path = self.files[self.idx]
        p = o3d.io.read_point_cloud(path)
        pts = np.asarray(p.points)
        d = np.linalg.norm(pts, axis=1)
        mask = (d >= self.args.min_range) & (d <= self.args.max_range)
        p.points = o3d.utility.Vector3dVector(pts[mask])
        colorize_by_distance(p, cmap_name=self.args.cmap, clip=tuple(self.args.clip), gamma=self.args.gamma)
        self.pcd.points = p.points
        self.pcd.colors = p.colors
        print(f"[{self.idx}/{len(self.files)-1}] {os.path.basename(path)} | pts: {len(p.points)}")
        self.vis.update_geometry(self.pcd)

    def next_frame(self, vis):
        self.idx = (self.idx + 1) % len(self.files)
        self.load_frame()
        return False

    def prev_frame(self, vis):
        self.idx = (self.idx - 1) % len(self.files)
        self.load_frame()
        return False

    def exit_and_save(self):
        if self.ctr:
            params = self.ctr.convert_to_pinhole_camera_parameters()
            o3d.io.write_pinhole_camera_parameters(self.session_json, params)
            print(f"Saved session camera parameters to {self.session_json}")
        self.vis.destroy_window()

    def run(self):
        self.vis.create_window(window_name="PCD Sequence Viewer", width=1280, height=720)
        self.vis.add_geometry(self.pcd)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(axes)
        self.ctr = self.vis.get_view_control()
        # Load saved camera
        if os.path.exists(self.session_json):
            params = o3d.io.read_pinhole_camera_parameters(self.session_json)
            self.ctr.convert_from_pinhole_camera_parameters(params)
        ro = self.vis.get_render_option()
        ro.point_size = 1.0
        ro.background_color = np.asarray([0.05, 0.05, 0.05])
        ro.show_coordinate_frame = True
        # Register callbacks
        self.vis.register_key_callback(262, self.next_frame)  # →
        self.vis.register_key_callback(263, self.prev_frame)  # ←
        # Show first
        self.load_frame()
        try:
            self.vis.run()
        finally:
            self.exit_and_save()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="View a sequence of PCDs")
    parser.add_argument('path', help='PCD file or directory')
    parser.add_argument('--min-range', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=40.0)
    parser.add_argument('--clip', nargs=2, type=float, default=[5, 95])
    parser.add_argument('--gamma', type=float, default=0.8)
    parser.add_argument('--cmap', type=str, default='turbo')
    args = parser.parse_args()
    files = list_pcd_files(args.path)
    base = os.path.splitext(os.path.basename(args.path.rstrip(os.sep)))[0]
    session_dir = os.path.dirname(args.path) if os.path.isfile(args.path) else args.path
    session_json = os.path.join(session_dir, f"{base}_cam.json")
    viewer = SequenceViewer(files, args, session_json)
    viewer.run()
