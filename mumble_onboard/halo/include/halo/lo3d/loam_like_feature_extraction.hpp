#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <unordered_set>

namespace halo {

class FeatureExtractor {
  public:
    static constexpr size_t SEGMENT_NUM = 6;
    struct Options {
        size_t neighbor_num                = 5;
        double min_ruggedness              = 0.1;
        double min_local_ruggedness        = 0.05;
        size_t max_edge_point_num_per_ring = 20;
        int num_lines                      = 16;
    };

    FeatureExtractor(const Options &o) : options_(o) {}
    ~FeatureExtractor() {}

    /**Workflow:
     * - extract_feature
        1. For each point in scan[5, size-5)
            1. calculate curvature:
                dx = sum(nearest 10 neighbors) - 10 * self.x
                curvature = dx^2 + dy^2 + dz^2
        2. seglength = total_points / 6;  for(i = 0; i < 6; i++)
            seg_start = i * seg_length
            seg_end = (i+1) * seg_length ?
            extract_feature_from_segment(seg_start, seg_end)

    Rings and Segments:
        ring0: |0,1,2,3|4,5,6,7|...
        ring1: |0,1,2,3|4,5,6,7|...
    IMPORTANT: the cloud must keep the ordering of points on the same ring.
     *
     */
    void extract(const PCLFullCloudPtr &full_cloud, PCLCloudXYZIPtr &edge_points, PCLCloudXYZIPtr &planar_points) const {
        if (full_cloud->points.size() <= 2 * options_.neighbor_num) {
            std::cerr << "Not enough points to extract features" << std::endl;
            return;
        }
        if (edge_points == nullptr) {
            edge_points.reset(new PCLCloudXYZI);
        }
        if (planar_points == nullptr) {
            planar_points.reset(new PCLCloudXYZI);
        }
        std::deque<std::deque<size_t>> ring_dqs(options_.num_lines);   // stores indices of full_cloud points
        for (size_t i = 0; i < full_cloud->size(); ++i) {
            const auto &pt = full_cloud->at(i);
            if (pt.ring >= options_.num_lines)
                assert(false && "ring number is larger than num_lines");
            ring_dqs.at(pt.ring).push_back(i);
        }

        for (const auto &ring_dq : ring_dqs) {
            size_t dq_size = ring_dq.size();
            std::vector<double> curvatures(dq_size);
            for (size_t i = options_.neighbor_num; i < dq_size - options_.neighbor_num; ++i) {
                // sum the neighbors
                double dx = 0, dy = 0, dz = 0;
                for (size_t j = i - options_.neighbor_num; j <= i + options_.neighbor_num; ++j) {
                    if (j == i)
                        continue;
                    size_t cloud_idx = ring_dq.at(j);
                    dx += full_cloud->points.at(cloud_idx).x;
                    dy += full_cloud->points.at(cloud_idx).y;
                    dz += full_cloud->points.at(cloud_idx).z;
                }
                size_t cloud_idx = ring_dq.at(i);
                dx -= full_cloud->points.at(cloud_idx).x * 2 * options_.neighbor_num;
                dy -= full_cloud->points.at(cloud_idx).y * 2 * options_.neighbor_num;
                dz -= full_cloud->points.at(cloud_idx).z * 2 * options_.neighbor_num;
                curvatures.at(i) = dx * dx + dy * dy + dz * dz;
            }
            // Divide the ring into 6 segments. First, find the start and end indices of each segment.
            size_t seg_length = dq_size / SEGMENT_NUM;
            for (size_t s = 0; s < SEGMENT_NUM; ++s) {
                size_t start_idx = s * seg_length;
                size_t end_idx   = (s == SEGMENT_NUM - 1) ? (dq_size - 1)
                                                          : ((s + 1) * seg_length - 1);

                // Output: indices of edge points, planar points of this segment, in this ring.
                std::deque<size_t> edge_pt_indices;
                std::deque<size_t> planar_pt_indices;
                extract_segment(start_idx, end_idx, ring_dq, curvatures, full_cloud, edge_pt_indices, planar_pt_indices);
                // Push points to the output clouds
                edge_points->points.reserve(edge_points->points.size() + edge_pt_indices.size());
                planar_points->points.reserve(planar_points->points.size() + planar_pt_indices.size());
                for (const auto &i : edge_pt_indices) {
                    size_t cloud_idx = ring_dq.at(i);
                    edge_points->points.push_back(to_pcl_point_xyzi(full_cloud->points.at(cloud_idx)));
                }
                for (const auto &i : planar_pt_indices) {
                    size_t cloud_idx = ring_dq.at(i);
                    planar_points->points.push_back(to_pcl_point_xyzi(full_cloud->points.at(cloud_idx)));
                }
            }
        }
    }

  private:
    /** Workflow:
     * - extract_feature_from_segment:
        1. Sort points by curvature, (from low to high)
        2. Find Edge points - start from the end (largest curvature):
            1. Reject curvatures less than 0.1
            2. Add point to edge_list, and the ignored_list
            3. Break if edge points have exceed a threshold
            4. Non-Maximum-Suppresion: Check the left 5 neighbors.
                1. Calculate its ruggedness of [k-1], [k].
                2. If ruggedness < threshold, add to ignored_list
                    - Otherwise, break (for further analysis)
            5. Repeat for the right 5 neghbours
        3. Find Planar Points - loop over the curvature list
            1. If the point is not in the ignored list, add to the surface_list
    Curvatures and Segment:
        curvature of: |0,1,2,3|4,5,6,7|...
        segment: |4,5,6,7|
     */
    void extract_segment(size_t start_idx, size_t end_idx,
                         const std::deque<size_t> &ring_dq,
                         const std::vector<double> &curvatures, const PCLFullCloudPtr full_cloud,
                         std::deque<size_t> &edge_pt_indices, std::deque<size_t> &planar_pt_indices) const {
        // Step 1: Set up: ignored_list, index array
        // sorted_indices: [start_idx, ...  end_idx] of ring
        std::vector<size_t> sorted_indices(end_idx - start_idx + 1);
        std::iota(sorted_indices.begin(), sorted_indices.end(), start_idx);
        // ascending array
        std::sort(sorted_indices.begin(), sorted_indices.end(),
                  [&curvatures](size_t a, size_t b) { return curvatures[a] < curvatures[b]; });
        std::vector<bool> ignored_indices(ring_dq.size(), false);
        // start from the end of the sorted array. Using iterator to avoid indexing confusion
        auto ignore_flat_neighbors = [&](bool check_right_neighbors, size_t idx) {   // TODO   Will this be created as a lambda, every time?
            // neighbors are in the range of ring[idx - 5, idx + 5]
            for (size_t off = 1; off <= options_.neighbor_num; ++off) {
                int main_off   = off;
                int neighb_off = off - 1;
                if (check_right_neighbors) {
                    main_off   = -main_off;
                    neighb_off = -neighb_off;
                }
                size_t pt1_idx = ring_dq.at(idx + main_off);
                size_t pt2_idx = ring_dq.at(idx + neighb_off);
                double l       = local_ruggedness(
                          full_cloud->points.at(pt1_idx),
                          full_cloud->points.at(pt2_idx));
                if (l > options_.min_local_ruggedness)
                    break;
                ignored_indices.at(idx + main_off) = true;
            }
        };
        // Step 2: Get the edge points
        for (auto rit = sorted_indices.rbegin(); rit != sorted_indices.rend(); ++rit) {
            size_t idx = *rit;
            if (curvatures[idx] < options_.min_ruggedness)
                break;
            if (ignored_indices.at(idx))
                continue;   // already ignored
            if (edge_pt_indices.size() > options_.max_edge_point_num_per_ring)
                break;
            edge_pt_indices.push_back(idx);
            ignored_indices.at(idx) = true;
            // check local neighbor, from m = idx + 1 to m = idx + 5: local_ruggedness = (pt[m].x - pt[m+1].x)^2 + ... if local_ruggedness < threshold, break
            // check local neighbor, from m = idx - 1 to m = idx - 5: (pt[m].x - pt[m-1].x)^2 + ...
            ignore_flat_neighbors(true, idx);
            ignore_flat_neighbors(false, idx);
        }

        // Step 3: Get the planar points
        for (size_t idx : sorted_indices) {
            if (ignored_indices.at(idx))
                continue;
            planar_pt_indices.push_back(idx);
        }
    }
    double local_ruggedness(const PCLFullPointType &pt1, const PCLFullPointType &pt2) const {
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        double dz = pt1.z - pt2.z;
        return dx * dx + dy * dy + dz * dz;
    }
    Options options_;
};

}   // namespace halo