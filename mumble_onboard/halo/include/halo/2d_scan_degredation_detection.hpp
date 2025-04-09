#pragma once

// Inspired by: https://zhuanlan.zhihu.com/p/642853423
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace halo {

bool detect_2d_degradation(std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr) {
    PCLCloud3DPtr cloud = laser_scan_2_PointXYZ(current_scan_ptr);
    PCLCloud3DPtr cloud_filtered(new pcl::PointCloud<PCLPoint3D>);

    pcl::StatisticalOutlierRemoval<PCLPoint3D> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);              // 对每个点分析的邻近点个数
    sor.setStddevMulThresh(1.0);   // 标准差倍数
    sor.filter(*cloud_filtered);

    pcl::search::KdTree<PCLPoint3D>::Ptr kdtree_;   // (new pcl::search::KdTree<PCLPoint3D>)
    kdtree_ = std::make_shared<pcl::search::KdTree<PCLPoint3D>>();
    kdtree_->setInputCloud(cloud_filtered);   // filtered cloud is our source of KD Tree

    pcl::EuclideanClusterExtraction<PCLPoint3D> clusterExtractor_;
    // Vector that stores clustering results
    std::vector<pcl::PointIndices> cluster_indices;
    clusterExtractor_.setClusterTolerance(0.1);
    clusterExtractor_.setMinClusterSize(10);      // each cluster contains at least 10 points
    clusterExtractor_.setMaxClusterSize(1000);    // each cluster contains at most 1000 points
    clusterExtractor_.setSearchMethod(kdtree_);   // Find nearby neighbors using KD Tree
    clusterExtractor_.setInputCloud(cloud_filtered);
    clusterExtractor_.extract(cluster_indices);

    CloudViewer<PCLPoint3D> cloud_viewer;
    auto viewer = cloud_viewer.get_viewer();

    int clusterNumber = 1;   // 输出聚类结果
    std::vector<Vec3f> line_coeffs;
    for (const auto &indices : cluster_indices) {
        std::cout << "Cluster " << clusterNumber << " has " << indices.indices.size() << " points." << std::endl;
        pcl::PointCloud<PCLPoint3D>::Ptr cluster(new pcl::PointCloud<PCLPoint3D>);
        pcl::copyPointCloud(*cloud_filtered, indices, *cluster);
        double r = static_cast<double>(rand()) / RAND_MAX;
        double g = static_cast<double>(rand()) / RAND_MAX;
        double b = static_cast<double>(rand()) / RAND_MAX;

        std::string clusterId = "cluster_" + std::to_string(clusterNumber);
        viewer->addPointCloud<PCLPoint3D>(cluster, clusterId);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, clusterId);
        clusterNumber++;

        // Calculate line coeffs if the point cloud size is larger than 10
        if (cluster->size() > 10) {
            line_coeffs.emplace_back(math::fit_line_2d(cluster));
        }
    }

    size_t same_dir_count = 0;
    for (size_t i = 0; i < line_coeffs.size(); ++i) {
        const auto &l1 = line_coeffs.at(i);
        float a1 = l1[0], b1 = l1[1];
        for (size_t j = i + 1; j < line_coeffs.size(); ++j) {
            const auto &l2 = line_coeffs.at(j);
            float a2 = l2[0], b2 = l2[1];
            if (std::fabs(a1 * b2 - a2 * b1) < 1e-1) {
                same_dir_count++;
            }
        }
    }
    std::cout << "same_dir_count" << same_dir_count << std::endl;

    float line_combo_num = (clusterNumber * (clusterNumber - 1)) / 2;
    if (line_combo_num * 2 / 3.0 > same_dir_count) {
        std::cout << "Scan has no degredation" << std::endl;
        return false;
    } else {
        std::cout << "Scan has degredation" << std::endl;
        return true;
    }
    // // Keep the viewer running until the user closes the window.
    // viewer->spin();
    // // Reset the viewer to avoid potential segfaults.
    // viewer.reset();
}
};   // namespace halo
