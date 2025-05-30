#!/usr/bin/env python3
"""
PCD Sequence Viewer with colorization and arrow-key stepping. Saves a single
camera-parameters JSON on exit, named after the input path (folder or file).
ESC key closes and triggers JSON save.

Usage:
    python3 pcd_sequence_viewer.py <path_to_pcd_or_folder> \
        [--min-range 0.3] [--max-range 40.0] [--clip 5 95] [--gamma 0.8] [--cmap turbo] [--reconstruct]

Features:
- Lists .pcd files in a folder or opens a single file
- Numerically sorts frames by trailing index
- Distance-based filtering and colorization
- Arrow keys (← / →) to step frames with wrap-around
- ESC to exit and save camera parameters to a single JSON
"""
import sys
import re
from pathlib import Path
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import colormaps
get_cmap = colormaps.get_cmap


def colorize_by_distance(
    pcd: o3d.geometry.PointCloud,
    cmap_name: str = "turbo",
    clip: tuple[float, float] = (5, 95),
    gamma: float = 0.8,
    cmap_range: tuple[float, float] = (0.2, 1.0),
) -> None:
    """
    Colorize a point cloud by distance using a colormap.
    """
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return

    dists = np.linalg.norm(pts, axis=1)
    lo, hi = np.percentile(dists, clip)
    d_clip = np.clip(dists, lo, hi)
    normalized = ((d_clip - lo) / (hi - lo + 1e-8)) ** gamma
    lo_factor, hi_factor = cmap_range
    scaled = lo_factor + (hi_factor - lo_factor) * normalized
    colors = get_cmap(cmap_name)(scaled)[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)


def list_pcd_files(path: Path) -> list[Path]:
    """
    List and sort .pcd files in a directory or single file.
    Numeric suffixes come first, then custom tags.
    When a single file is provided, list all siblings and use that file as the start.
    """
    if path.is_dir():
        files = list(path.glob("*.pcd"))
    elif path.is_file() and path.suffix.lower() == ".pcd":
        # list all .pcd in the same directory
        files = sorted(path.parent.glob("*.pcd"))
    else:
        sys.exit(f"No .pcd files found at {path}")

    suffix_order = ["after_optim", "before_optim", "after_optim_raw"]

    def sort_key(fp: Path):
        name = fp.stem
        parts = name.split('_')
        last = parts[-1]
        if last.isdigit():
            prefix = "_".join(parts[:-1])
            return (0, prefix, int(last))
        order_idx = suffix_order.index(last) if last in suffix_order else len(suffix_order)
        return (1, name, order_idx)

    return sorted(files, key=sort_key)


class SequenceViewer:
    """
    Visualize a sequence of PCD files with coloring and optional reconstruction.
    """

    def __init__(
        self,
        files: list[Path],
        args: argparse.Namespace,
        session_json: Path,
    ) -> None:
        self.files = files
        self.args = args
        self.session_json = session_json
        # determine starting index: if a file was passed, start at its position
        self.idx = 0
        if args.path.is_file() and args.path.suffix.lower() == ".pcd":
            try:
                self.idx = files.index(args.path)
            except ValueError:
                self.idx = 0
        self.pcd = o3d.geometry.PointCloud()
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.ctr = None

    def load_frame(self) -> None:
        pcd_path = self.files[self.idx]
        cloud = o3d.io.read_point_cloud(str(pcd_path))
        pts = np.asarray(cloud.points)
        dists = np.linalg.norm(pts, axis=1)
        mask = (dists >= self.args.min_range) & (dists <= self.args.max_range)
        cloud.points = o3d.utility.Vector3dVector(pts[mask])
        colorize_by_distance(
            cloud,
            cmap_name=self.args.cmap,
            clip=tuple(self.args.clip),
            gamma=self.args.gamma,
        )

        print(f"[{self.idx}/{len(self.files)-1}] {pcd_path.name} | pts: {len(cloud.points)}")

        if self.args.reconstruct:
            # ensure normals for Poisson
            cloud.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )
            cloud.normalize_normals()
            cloud.orient_normals_consistent_tangent_plane(k=100)
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                cloud, depth=8, linear_fit=True
            )
            densities = np.asarray(densities)

            # color vertices by density
            norm = (densities - densities.min()) / (densities.max() - densities.min())
            cmap = get_cmap("viridis")
            mesh.vertex_colors = o3d.utility.Vector3dVector(cmap(norm)[:, :3])

            # prune triangles: keep only those whose vertices all exceed density threshold
            vert_mask = densities > np.quantile(densities, 0.01)
            triangles = np.asarray(mesh.triangles)
            tri_mask = vert_mask[triangles].all(axis=1)
            mesh.remove_triangles_by_mask(~tri_mask)
            mesh.remove_unreferenced_vertices()

            # smooth mesh
            mesh = mesh.filter_smooth_laplacian(number_of_iterations=10, lambda_filter=0.5)
            mesh.compute_vertex_normals()

            # merge small patches via vertex clustering
            if self.args.cluster_size > 0:
                mesh = mesh.simplify_vertex_clustering(
                    voxel_size=self.args.cluster_size
                )
                mesh.compute_vertex_normals()

            o3d.visualization.draw_geometries([mesh])

        self.pcd.points = cloud.points
        self.pcd.colors = cloud.colors
        self.vis.update_geometry(self.pcd)

    def next_frame(self, vis) -> bool:
        self.idx = (self.idx + 1) % len(self.files)
        self.load_frame()
        return False

    def prev_frame(self, vis) -> bool:
        self.idx = (self.idx - 1) % len(self.files)
        self.load_frame()
        return False

    def exit_and_save(self) -> None:
        if self.ctr:
            params = self.ctr.convert_to_pinhole_camera_parameters()
            o3d.io.write_pinhole_camera_parameters(str(self.session_json), params)
            print(f"Saved camera parameters to {self.session_json}")
        self.vis.destroy_window()

    def run(self) -> None:
        self.vis.create_window("PCD Sequence Viewer", width=1280, height=720)
        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))
        self.ctr = self.vis.get_view_control()
        self.ctr.set_constant_z_near(0.05)
        self.ctr.set_constant_z_far(1000.0)

        if self.session_json.exists():
            params = o3d.io.read_pinhole_camera_parameters(str(self.session_json))
            self.ctr.convert_from_pinhole_camera_parameters(params)

        ro = self.vis.get_render_option()
        ro.light_on = False
        ro.point_size = 1.5
        ro.background_color = np.array([0.05, 0.05, 0.05])
        ro.show_coordinate_frame = True

        self.vis.register_key_callback(262, self.next_frame)
        self.vis.register_key_callback(263, self.prev_frame)

        self.load_frame()
        try:
            self.vis.run()
        finally:
            self.exit_and_save()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="View a sequence of PCDs with optional reconstruction"
    )
    parser.add_argument('path', type=Path, help='PCD file or directory')
    parser.add_argument('--min-range', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=40.0)
    parser.add_argument('--clip', nargs=2, type=float, default=[5, 95])
    parser.add_argument('--gamma', type=float, default=0.8)
    parser.add_argument('--cmap', type=str, default='turbo')
    parser.add_argument('--reconstruct', action='store_true', help='Enable Poisson reconstruction')
    parser.add_argument('--cluster-size', type=float, default=0.4, help='Voxel size for vertex clustering to merge small patches')

    args = parser.parse_args()
    files = list_pcd_files(args.path)
    base = args.path.stem
    session_json = (args.path.parent / f"{base}_cam.json") if args.path.is_file() else (args.path / f"{base}_cam.json")

    viewer = SequenceViewer(files, args, session_json)
    viewer.run()
