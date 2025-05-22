#!/usr/bin/env python3
"""
PCD Sequence Viewer with colorization and arrow-key stepping. Saves a single
camera-parameters JSON on exit, named after the input path (folder or file).

Usage:
    python3 pcd_sequence_viewer.py <path_to_pcd_or_folder> \
        [--min-range 0.3] [--max-range 40.0] [--clip 5 95] [--gamma 0.8] [--cmap turbo] [-s]

Features:
- Lists .pcd files in a folder or opens a single file
- Numerically sorts frames by trailing index
- Distance-based filtering and colorization
- Arrow keys (← / →) to step frames
- On exit (Esc or window close), saves one JSON with camera parameters
  to <basename>_cam.json (folder basename for directories, or filename basename)
"""
import os
import sys
import re
import argparse
import numpy as np
import open3d as o3d
from matplotlib.cm import get_cmap

# Colorization utility
def colorize_by_distance(
        pcd,
        cmap_name="turbo",
        clip=(5, 95),
        gamma=0.8,
        cmap_range=(0.2, 1.0)
    ):
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return
    dists = np.linalg.norm(pts, axis=1)
    lo, hi = np.percentile(dists, clip)
    d_clip = np.clip(dists, lo, hi)
    dn = (d_clip - lo) / (hi - lo + 1e-8)
    dn = dn ** gamma
    cmap_lo, cmap_hi = cmap_range
    dn = cmap_lo + (cmap_hi - cmap_lo) * dn
    colors = get_cmap(cmap_name)(dn)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)

# List .pcd files in folder or single file, numerically sorted by trailing index
def list_pcd_files(path):
    if os.path.isdir(path):
        files = [os.path.join(path, f)
                 for f in os.listdir(path)
                 if f.lower().endswith('.pcd')]
        # numeric sort
        def idx_key(f):
            m = re.search(r'_(\d+)\.pcd$', f)
            return int(m.group(1)) if m else -1
        files.sort(key=idx_key)
        return files
    elif os.path.isfile(path) and path.lower().endswith('.pcd'):
        return [path]
    else:
        print(f"No .pcd files found at {path}")
        sys.exit(1)

class SequenceViewer:
    def __init__(self, files, args, session_json):
        self.files = files
        self.idx = 0
        self.args = args
        self.session_json = session_json
        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.ctr = None

    def load_frame(self):
        path = self.files[self.idx]
        p = o3d.io.read_point_cloud(path)
        # Range filter
        pts = np.asarray(p.points)
        d = np.linalg.norm(pts, axis=1)
        mask = (d >= self.args.min_range) & (d <= self.args.max_range)
        p.points = o3d.utility.Vector3dVector(pts[mask])
        # Colorize
        colorize_by_distance(p,
            cmap_name=self.args.cmap,
            clip=tuple(self.args.clip),
            gamma=self.args.gamma
        )
        # Update
        self.pcd.points = p.points
        self.pcd.colors = p.colors
        # Restore per-frame camera if exists
        cam_file = os.path.splitext(path)[0] + '_cam.json'
        if os.path.exists(cam_file) and self.ctr:
            params = o3d.io.read_pinhole_camera_parameters(cam_file)
            self.ctr.convert_from_pinhole_camera_parameters(params)
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        print(f"[{self.idx}/{len(self.files)-1}] {os.path.basename(path)} | pts: {len(p.points)}")

    def next(self, vis):
        if self.idx < len(self.files) - 1:
            self.idx += 1
            self.load_frame()
        return False

    def prev(self, vis):
        if self.idx > 0:
            self.idx -= 1
            self.load_frame()
        return False

    def run(self):
        self.vis.create_window(
            window_name="PCD Sequence Viewer",
            width=1280, height=720,
            left=50, top=50
        )
        self.vis.add_geometry(self.pcd)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.vis.add_geometry(axes)
        self.ctr = self.vis.get_view_control()
        ro = self.vis.get_render_option()
        ro.point_size = 1.0
        ro.background_color = np.asarray([0.05, 0.05, 0.05])
        ro.show_coordinate_frame = True
        # Key callbacks
        self.vis.register_key_callback(262, self.next)  # →
        self.vis.register_key_callback(263, self.prev)  # ←
        # Load first frame
        self.load_frame()
        try:
            self.vis.run()
        finally:
            # On exit, save session camera
            params = self.ctr.convert_to_pinhole_camera_parameters()
            o3d.io.write_pinhole_camera_parameters(self.session_json, params)
            print(f"Saved session camera parameters to {self.session_json}")
            self.vis.destroy_window()

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
    # Determine session JSON path
    base = args.path.rstrip('/').split(os.sep)[-1]
    session_json = os.path.join(
        os.path.dirname(args.path) if os.path.isfile(args.path) else args.path,
        f"{base}_cam.json"
    )
    viewer = SequenceViewer(files, args, session_json)
    viewer.run()
