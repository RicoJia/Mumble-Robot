#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/icp_3d_methods.hpp>
#include <halo/lo3d/ndt_3d.hpp>

// EPFL 雕像数据集：./ch7/EPFL/aquarius_{sourcd.pcd, target.pcd}，真值在对应目录的_pose.txt中
// EPFL 模型比较精细，配准时应该采用较小的栅格
std::string source_path       = "./data/ch7/EPFL/kneeling_lady_source.pcd";   // first point cloud
std::string target_path       = "./data/ch7/EPFL/kneeling_lady_target.pcd";   // second point cloud
std::string ground_truth_path = "./data/ch7/EPFL/kneeling_lady_pose.txt";     // ground truth path
using halo::operator<<;

void parse_args(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if ((arg == "--source_path" || arg == "-s") && i + 1 < argc) {
            source_path = argv[i + 1];   // Store the next argument as source_path
            ++i;                         // Skip the next argument since it's the value
        } else if ((arg == "--target_path" || arg == "-t") && i + 1 < argc) {
            target_path = argv[i + 1];   // Store the next argument as source_path
            ++i;                         // Skip the next argument since it's the value
        } else if ((arg == "--ground_truth_path" || arg == "-g") && i + 1 < argc) {
            ground_truth_path = argv[i + 1];   // Store the next argument as source_path
            ++i;                               // Skip the next argument since it's the value
        }
    }
    std::cout << "source_path: " << source_path << ", target_path: " << target_path << std::endl;
}

class ICP3DTest : public ::testing::Test {
  protected:
    halo::PCLCloudXYZIPtr source;
    halo::PCLCloudXYZIPtr target;
    halo::SE3 ground_truth_pose;
    halo::SE3 relative_pose;
    bool success          = false;
    std::string test_name = "";

    void SetUp() override {
        source.reset(new halo::PCLCloudXYZI);
        target.reset(new halo::PCLCloudXYZI);
        pcl::io::loadPCDFile(source_path, *source);
        pcl::io::loadPCDFile(target_path, *target);

        std::ifstream in_stream(ground_truth_path);
        if (in_stream) {
            double x, y, z, qx, qy, qz, qw;
            in_stream >> x >> y >> z >> qw >> qx >> qy >> qz;
            ground_truth_pose.translation() = halo::Vec3d(x, y, z);
            ground_truth_pose.so3()         = halo::SO3(Eigen::Quaterniond(qw, qx, qy, qz));
        }
    }

    void TearDown() override {
        // TODO
        std::cout << "2" << std::endl;
        if (success) {
            std::cout << "Alignment success!" << std::endl;
            double pose_error = (ground_truth_pose.inverse() * relative_pose).log().norm();
            std::cout << " pose error: " << pose_error << std::endl;
            save_pcl(relative_pose, test_name);
        } else {
            std::cout << "Alignment failed!" << std::endl;
        }
        std::cout << "relative_pose: " << relative_pose << std::endl;
        std::cout << "ground truth: " << ground_truth_pose << std::endl;
        source.reset();
        target.reset();
    }

    void save_pcl(halo::SE3 relative_pose, const std::string &test_name) {
        halo::PCLCloudXYZIPtr merged_aligned_cloud(new halo::PCLCloudXYZI);
        *merged_aligned_cloud = *source + *target;   // Merge clouds
        halo::save_pcd_file("/tmp/" + test_name + "_pcl_merged_unaligned.pcd", *merged_aligned_cloud);

        halo::PCLCloudXYZIPtr aligned_cloud(new halo::PCLCloudXYZI);
        pcl::transformPointCloud(*source, *aligned_cloud, relative_pose.matrix().cast<float>());

        // Merge aligned with target
        halo::PCLCloudXYZIPtr merged_cloud(new halo::PCLCloudXYZI);
        *merged_cloud = *aligned_cloud + *target;   // Merge clouds
        // Save merged point cloud
        halo::save_pcd_file("/tmp/" + test_name + "_pcl_merged.pcd", *merged_cloud);
    }
};

TEST_F(ICP3DTest, Test3DICP_Ptpt) {
    halo::profile_and_call(
        [&]() {
            std::cout << "====================== Test3DICP_Ptpt ======================" << std::endl;
            halo::ICP3D::Options options;
            halo::ICP3D icp_3d(options);
            icp_3d.set_source(source);
            icp_3d.set_target(target);
            success   = icp_3d.pt_pt_icp3d(relative_pose);
            test_name = "icp_pt_pt_pcl";
        });
}

TEST_F(ICP3DTest, Test3DICP_PtLine) {
    halo::profile_and_call(
        [&]() {
            std::cout << "====================== Test3DICP_PtLine ======================" << std::endl;
            halo::ICP3D::Options options;
            halo::ICP3D icp_3d(options);
            icp_3d.set_source(source);
            icp_3d.set_target(target);
            success   = icp_3d.pt_plane_icp3d(relative_pose);
            test_name = "icp_pt_line_pcl";
        });
}

TEST_F(ICP3DTest, Test3DICP_Ptplane) {
    halo::profile_and_call(
        [&]() {
            std::cout << "====================== Test3DICP_PtPlane ======================" << std::endl;
            halo::ICP3D::Options options;
            halo::ICP3D icp_3d(options);
            icp_3d.set_source(source);
            icp_3d.set_target(target);
            success   = icp_3d.pt_plane_icp3d(relative_pose);
            test_name = "icp_pt_plane_pcl";
        });
}

TEST_F(ICP3DTest, Test3DNDT) {
    halo::profile_and_call(
        [&]() {
            std::cout << "====================== Test3D NDT ======================" << std::endl;
            halo::NDT3DOptions options;
            options.remove_centroid_ = true;
            // halo::NDT3D<halo::NeighborCount::NEARBY6> ndt_3d(options);
            halo::NDT3D<halo::NeighborCount::CENTER> ndt_3d(options);
            ndt_3d.set_source(source);
            ndt_3d.set_target(target);
            success   = ndt_3d.align_gauss_newton(relative_pose);
            test_name = "ndt_pcl";
        });
}

// DANGER: commented out because there's a bug likely in PCL that causes a segfault, right at when the
// TEST_F(ICP3DTest, Test3DPCL_ICP) {
//     halo::profile_and_call(
//         [&]() {
//             std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
//             pcl::IterativeClosestPoint<halo::PCLPointXYZI, halo::PCLPointXYZI> icp_pcl;
//             icp_pcl.setInputSource(source);
//             icp_pcl.setInputTarget(target);
//             halo::PCLCloudXYZIPtr output_pcl(new halo::PCLCloudXYZI);
//             icp_pcl.align(*output_pcl);

//             // Due to numerical error, R might not be orthogonal.
//             Eigen::Matrix4d T = icp_pcl.getFinalTransformation().cast<double>();
//             // Extract the top-left 3x3 rotation block
//             Eigen::Matrix3d R = T.block<3, 3>(0, 0);
//             // Re-orthogonalize R using SVD
//             Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
//             Eigen::Matrix3d R_orthogonal = svd.matrixU() * svd.matrixV().transpose();
//             T.block<3, 3>(0, 0) = R_orthogonal;

//             relative_pose = halo::SE3(T);
//             std::cout<<"pose error: "<<(ground_truth_pose.inverse() * relative_pose).log().norm()<<std::endl;
//             success   = true;
//             test_name = "pcl_icp_pcl";
//             std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double, std::milli> duration = end - start;
//             std::cout << test_name << " took " << duration.count() << " ms" << std::endl;
//         });
// }

// DANGER: commented out because there's a bug likely in PCL that causes a segfault, right at when the
// the Lambda is done
// TEST_F(ICP3DTest, Test3DPCL_NDT) {
//     halo::profile_and_call(
//         [&]() {
//             std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
//             pcl::NormalDistributionsTransform<halo::PCLPointXYZI, halo::PCLPointXYZI> ndt_pcl;
//             ndt_pcl.setInputSource(source);

//             assert(target && "target point cloud is null!");
//             std::cout << "Target cloud has " << target->points.size() << " points." << std::endl;
//             ndt_pcl.setInputTarget(this->target); // with this line, the test crashes
//             halo::PCLCloudXYZIPtr output_pcl(new halo::PCLCloudXYZI);
//             ndt_pcl.align(*output_pcl);
//             // Due to numerical error, R might not be orthogonal.
//             Eigen::Matrix4d T = ndt_pcl.getFinalTransformation().cast<double>();
//             // Extract the top-left 3x3 rotation block
//             Eigen::Matrix3d R = T.block<3, 3>(0, 0);
//             // Re-orthogonalize R using SVD
//             Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
//             Eigen::Matrix3d R_orthogonal = svd.matrixU() * svd.matrixV().transpose();
//             T.block<3, 3>(0, 0) = R_orthogonal;

//             relative_pose = halo::SE3(T);
//             std::cout<<"pose error: "<<(ground_truth_pose.inverse() * relative_pose).log().norm()<<std::endl;
//             success   = true;
//             test_name = "pcl_ndt_pcl";
//             std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double, std::milli> duration = end - start;
//             std::cout << test_name << " took " << duration.count() << " ms" << std::endl;
//         });
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    parse_args(argc, argv);

    return RUN_ALL_TESTS();
}