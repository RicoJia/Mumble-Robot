#pragma once

#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <halo/common/point_cloud_processing.hpp>

namespace halo {
class PCLMapViewer {
  public:
    PCLMapViewer(const float &leaf_size, bool viz_on_car, size_t viz_speedups = 1)
        : leaf_size_(leaf_size), tmp_cloud_(new PCLCloudXYZI), local_map_(new PCLCloudXYZI),
          viz_on_car_(viz_on_car), viz_speedups_(viz_speedups) {
        viewer_.reset(new pcl::visualization::PCLVisualizer());
        viewer_->addCoordinateSystem(10, "world");
        voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
        // PCL 1.14 does not like VTK 9.1
        vtkObject::GlobalWarningDisplayOff();
    }

    void SetPoseAndCloudNoViz(const SE3 &pose, PCLCloudXYZIPtr cloud_world) {
        voxel_filter_.setInputCloud(cloud_world);
        voxel_filter_.filter(*tmp_cloud_);

        *local_map_ += *tmp_cloud_;
        tmp_cloud_->clear();
        voxel_filter_.setInputCloud(local_map_);
        voxel_filter_.filter(*tmp_cloud_);
        // swap or copy the filtered result back into local_map_
        local_map_->swap(*tmp_cloud_);

        if (local_map_->size() > 600000) {
            leaf_size_ *= 1.26;
            voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            LOG(INFO) << "viewer set leaf size to " << leaf_size_;
        }
    }

    void set_spin_time(int spin_time) {
        spin_time_ = spin_time;
    }

    /**
     * 增加一个Pose和它的点云（世界系）
     * @param pose
     * @param cloud_world
     */
    void SetPoseAndCloud(const SE3 &pose, PCLCloudXYZIPtr cloud_world) {
        LapseTimer timer;
        voxel_filter_.setInputCloud(cloud_world);
        voxel_filter_.filter(*tmp_cloud_);

        *local_map_ += *tmp_cloud_;
        tmp_cloud_->clear();
        voxel_filter_.setInputCloud(local_map_);
        voxel_filter_.filter(*tmp_cloud_);
        // swap or copy the filtered result back into local_map_
        local_map_->swap(*tmp_cloud_);
        cnt_frame_++;

        if (viewer_ != nullptr && cnt_frame_ % viz_speedups_ == 0) {
            viewer_->removePointCloud("local_map");
            viewer_->removeCoordinateSystem("vehicle");
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldColor(local_map_, "z");
            viewer_->addPointCloud<PCLPointXYZI>(local_map_, fieldColor, "local_map");

            Eigen::Affine3f T;
            T.matrix() = pose.matrix().cast<float>();
            viewer_->addCoordinateSystem(5, T, "vehicle");
            if (viz_on_car_) {
                // --- Bird’s‑Eye Camera Setup ---
                Eigen::Vector3f vehicle_pos = T.translation();
                Eigen::Vector3f cam_pos     = vehicle_pos + Eigen::Vector3f(0, 0, 150);   // in meters
                Eigen::Vector3f view_pt     = vehicle_pos;
                Eigen::Vector3f up_dir      = Eigen::Vector3f(0, 1, 0);

                viewer_->setCameraPosition(
                    cam_pos.x(), cam_pos.y(), cam_pos.z(),
                    view_pt.x(), view_pt.y(), view_pt.z(),
                    up_dir.x(), up_dir.y(), up_dir.z());
            }
            viewer_->spinOnce(spin_time_);
            cnt_frame_ = 0;
        }

        if (local_map_->size() > 600000) {
            leaf_size_ *= 1.26;
            voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            LOG(INFO) << "viewer set leaf size to " << leaf_size_;
        }
    }

    /// 存储地图至PCD文件
    void SaveMap(std::string path) {
        if (local_map_->size() > 0) {
            save_pcd_file<PCLCloudXYZI>(path, *local_map_);   // TODO: 这里需要改成异步存储
            LOG(INFO) << "save map to " << path;
        } else {
            LOG(INFO) << "map 是空的" << path;
        }
    }

    void Clean() {
        tmp_cloud_->clear();
        local_map_->clear();
    }

    void ClearAndResetLeafSize(const float leaf_size) {
        leaf_size_ = leaf_size;
        local_map_->clear();
        voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    }

  private:
    pcl::VoxelGrid<PCLPointXYZI> voxel_filter_;
    pcl::visualization::PCLVisualizer::Ptr viewer_ = nullptr;   // pcl viewer
    float leaf_size_                               = 1.0;
    PCLCloudXYZIPtr tmp_cloud_;   //
    PCLCloudXYZIPtr local_map_;
    bool viz_on_car_     = false;   // 是否在车上可视化
    size_t viz_speedups_ = 1;       // 可视化速度加速倍数
    size_t cnt_frame_    = 0;       // 计数器
    int spin_time_       = 1;
};
}   // namespace halo
