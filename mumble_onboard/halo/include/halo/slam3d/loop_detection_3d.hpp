#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <pcl/registration/ndt.h>

#include <halo/slam3d/frontend_3d.hpp>
#include <halo/common/yaml_loaded_config.hpp>

#include <string>
#include <memory>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>   // << include the ROS2 msg

// Keyframe: need id
// 2. remove_groud
// Build submap: can we use gaussian voxel instead?

namespace halo {
struct LoopCandidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t idx1_ = 0;
    size_t idx2_ = 0;
    SE3 T12_;
    double ndt_score_ = 0.0;
};

// TODO: I want to make the pointers const, Is there a good way?
class LoopDetection3D {
    using KeyFrame3DPtr = std::shared_ptr<KeyFrame3D>;   // This should be const

  public:
    explicit LoopDetection3D(const std ::string &config_yaml) {
        options_.add_option<int>("min_id_interval", 50);
        options_.add_option<double>("min_distance", 30.0);
        options_.add_option<int>("skip_id", 5);
        options_.add_option<double>("ndt_score_th", 2.5);
        options_.add_option<double>("ground_z_val", 0.2);
        options_.add_option<int>("consecutive_idx_check_range", 10);
        options_.add_option<int>("loop_closure_submap_interval", 2);

        options_.add_option<double>("loop_detection_ndt_epsilon", 0.05);
        options_.add_option<double>("loop_detection_ndt_step_size", 0.5);
        options_.add_option<bool>("save_loop_detection_pcd", false);

        options_.load_from_yaml(config_yaml);
    }

    void run(std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        std::cout << "Detecting loop candidates..." << std::endl;
        detect_loop_candidates(keyframes_ptr);
        compute_loop_candidates(keyframes_ptr);
    }

    std::deque<LoopCandidate> *get_successful_candidates() {
        return &successful_loop_candidates_;
    }

  private:
    /**
     * @brief go through all keyframes, and find loop candidates
     * Workflow:
     * 1. Reject keyframes whose IDs too close to each other
     * 2. Accept keyframes that are close to each other
     *
     * @param keyframes_ptr
     */
    void detect_loop_candidates(std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        // TODO: in Gao's implementation, if we recently already have an iter_first and iter_second,
        // We would skip their neighbors. That could speed things up

        // TODO - test code, must remove
        if (keyframes_ptr->size() >= 2) {
            // for (int i = 0; i < keyframes_ptr->size() - 1; ++i) {
            //     for (int j = 10; i+j < keyframes_ptr->size() - 1; ++j) {
            auto kf1 = (*keyframes_ptr)[1];
            auto kf2 = (*keyframes_ptr)[38];

            // optional: still guard by your config thresholds
            Vec3d delta = kf1->lidar_pose_.translation() - kf2->lidar_pose_.translation();
            loop_candidates_.emplace_back(
                kf1->id_,   // id1
                kf2->id_,   // id2
                // use opti_pose_ instead of lidar_pose if you have it:
                kf1->lidar_pose_.inverse() * kf2->lidar_pose_,
                0.0   // ndt_score
            );
            //     }

            // }
        }

        // // Double for loop
        // for (auto iter_first = keyframes_ptr->begin(); iter_first != keyframes_ptr->end(); ++iter_first) {
        //     auto kf_first = *iter_first;

        //     for (auto iter_second = iter_first + 1; iter_second != keyframes_ptr->end(); ++iter_second) {
        //         auto kf_second = *iter_second;
        //         if (abs(int(kf_first->id_) - int(kf_second->id_)) < options_.get<int>("min_id_interval"))
        //             // If two IDs are too close, do not consider loop closure
        //             continue;

        //         // TODO: PLEASE USE opti_pose_1, instead of lidar_pose
        //         Vec3d dist = kf_first->lidar_pose_.translation() - kf_second->lidar_pose_.translation();
        //         if (dist.norm() < options_.get<double>("min_distance")) {
        //             loop_candidates_.emplace_back(
        //                 kf_first->id_,    // id1
        //                 kf_second->id_,   // id2
        //                 // TODO: PLEASE USE opti_pose_1, instead of lidar_pose
        //                 kf_first->lidar_pose_.inverse() * kf_second->lidar_pose_,   // Tij
        //                 0.0                                                         // ndt_score
        //             );
        //         }
        //     }
        // }
    }
    void compute_loop_candidates(std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        std::for_each(std::execution::par_unseq, loop_candidates_.begin(), loop_candidates_.end(),
                      [this, &keyframes_ptr](LoopCandidate &c) { check_candidate(c, keyframes_ptr); });

        for (const auto &c : loop_candidates_) {
            // //TODO
            // std::cout<<"c: "<<c.idx1_<<", "<<c.idx2_<<", score: "<<c.ndt_score_<<std::endl;
            const auto &kf1 = (*keyframes_ptr)[c.idx1_];
            const auto &kf2 = (*keyframes_ptr)[c.idx2_];

            // now you can print the frontend_id_ on each
            std::cout
                << "Loop candidate: " << c.idx1_
                << " (frontend_id=" << kf1->frontend_id_ << ")"
                << ", " << c.idx2_
                << " (frontend_id=" << kf2->frontend_id_ << ")"
                << ", ndt_score=" << c.ndt_score_
                << std::endl;

            // TODO: using < because we are using fitness score
            if (c.ndt_score_ < options_.get<double>("ndt_score_th")) {
                successful_loop_candidates_.emplace_back(c);
            }
        }
    }

    /**
     * @brief build submap, and compute NDT score
     * Workflow:
     * 1. Build submap for keyframe 1
     * 2. Use a pyramid pcl ndt to align keyframe 2 with submap. The result is the ndt score
     *
     * @param c
     * @param keyframes_ptr
     */
    void check_candidate(LoopCandidate &c,
                         const std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        KeyFrame3DPtr kf2 = keyframes_ptr->at(c.idx2_);

        auto build_submap = [&](int given_id) -> PCLCloudXYZIPtr {
            PCLCloudXYZIPtr submap(new PCLCloudXYZI);
            for (int idx = -options_.get<int>("consecutive_idx_check_range"); idx < options_.get<int>("consecutive_idx_check_range");
                 idx += options_.get<int>("loop_closure_submap_interval")) {
                int id = idx + given_id;
                if (id < 0 || id >= int(keyframes_ptr->size())) {
                    continue;
                }
                auto kf = keyframes_ptr->at(id);
                // TODO: TO FIX
                // auto cloud = remove_ground_cp(kf->cloud_, options_.get<double>("ground_z_val"));
                auto cloud = kf->cloud_;

                if (cloud->empty()) {
                    std::cerr << "Cloud is empty after removing ground points!" << std::endl;
                    continue;
                }

                // TODO: PLEASE USE opti_pose_1, instead of lidar_pose
                SE3 Twb = kf->lidar_pose_;
                PCLCloudXYZIPtr cloud_trans(new PCLCloudXYZI);
                pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());
                *submap += *cloud_trans;
            }
            return submap;
        };

        auto submap_kf1_orig = build_submap(c.idx1_);

        PCLCloudXYZIPtr submap_kf2_orig(new PCLCloudXYZI);
        pcl::copyPointCloud(*kf2->cloud_, *submap_kf2_orig);
        if (submap_kf1_orig->empty() || submap_kf2_orig->empty()) {
            c.ndt_score_ = 0;
            return;
        }

        // https://pcl.readthedocs.io/projects/tutorials/en/master/normal_distributions_transform.html
        pcl::NormalDistributionsTransform<PCLPointXYZI, PCLPointXYZI> ndt;
        // Δξk​=−H(ξk​)−1∇f(ξk​),
        // ξk+1​=ξk​+αk​Δξk​.
        // Convergence is when ∥Δξk​∥2​<ε, the epsilon here

        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon(options_.get<double>("loop_detection_ndt_epsilon"));   // 5 cm,
        // Setting scale dependent NDT parameters. For smaller objects like coffee cups, this needs to be smaller.
        ndt.setStepSize(options_.get<double>("loop_detection_ndt_step_size"));   // almost a full metre

        // In practice you don’t always take the full Newton step ΔξkΔξk​,
        // but perform a 1D line‐search to ensure stability.
        // You choose a step‐length αk  =  min⁡ ⁣(1,  αmax⁡/Δξk)
        // the number of Newton–type update steps.
        ndt.setMaximumIterations(40);

        // TODO: please use opti_pose_1, instead of lidar_pose
        Mat4f Tw2 = kf2->lidar_pose_.matrix().cast<float>();
        PCLCloudXYZIPtr output(new PCLCloudXYZI);
        std::vector<double> res{10.0, 5.0, 3.0, 1.0};

        ///////////////////////////////////////////////////////////////////////////////////////////
        // Save the submaps if necessary
        if (options_.get<bool>("save_loop_detection_pcd")) {
            const std::string base = std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_);
            // 3) transform the second submap by the final Tw2
            PCLCloudXYZIPtr transformed_submap2(new PCLCloudXYZI);
            pcl::transformPointCloud(
                *submap_kf2_orig,
                *transformed_submap2,
                Tw2   // world→world transform from NDT
            );
            // 4) merge with the first submap
            PCLCloudXYZIPtr merged(new PCLCloudXYZI);
            *merged = *submap_kf1_orig;        // copy submap1
            *merged += *transformed_submap2;   // append transformed submap2
            save_pcd_file("/tmp/loop_detection_results/", base + "_before_optim_merged.pcd", merged);
        }
        ///////////////////////////////////////////////////////////////////////////////////////////

        for (const auto &resolution : res) {
            // TODO
            std::cout << "Tw2: " << Tw2 << std::endl;

            PCLCloudXYZIPtr submap_kf1(new PCLCloudXYZI);
            pcl::copyPointCloud(*submap_kf1_orig, *submap_kf1);
            PCLCloudXYZIPtr submap_kf2(new PCLCloudXYZI);
            pcl::copyPointCloud(*submap_kf2_orig, *submap_kf2);

            ndt.setResolution(resolution);
            // need filtering
            downsample_point_cloud(submap_kf1, resolution * 0.1);
            downsample_point_cloud(submap_kf2, resolution * 0.1);

            ndt.setInputSource(submap_kf2);
            ndt.setInputTarget(submap_kf1);
            ndt.align(*output, Tw2);
            Tw2 = ndt.getFinalTransformation();
        }
        // TODO
        std::cout << "Tw2: " << Tw2 << std::endl;

        Mat4d T = Tw2.cast<double>();
        Quatd q(T.block<3, 3>(0, 0));
        q.normalize();
        Vec3d t = T.block<3, 1>(0, 3);
        c.T12_  = kf2->lidar_pose_.inverse() * SE3(q, t);
        // P(T)∝i=1∏Ns​​pi​(T), and this is pertinent to the final resolution
        // c.ndt_score_ = ndt.getTransformationLikelihood();
        // TODO:
        // Interpretation: A lower fitness score generally indicates a better alignment,
        c.ndt_score_ = ndt.getFitnessScore();

        ///////////////////////////////////////////////////////////////////////////////////////////
        // Save the submaps if necessary
        if (options_.get<bool>("save_loop_detection_pcd")) {
            const std::string base = std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_);
            // 3) transform the second submap by the final Tw2
            PCLCloudXYZIPtr transformed_submap2(new PCLCloudXYZI);
            pcl::transformPointCloud(
                *submap_kf2_orig,
                *transformed_submap2,
                Tw2   // world→world transform from NDT
            );
            // 4) merge with the first submap
            PCLCloudXYZIPtr merged(new PCLCloudXYZI);
            *merged = *submap_kf1_orig;        // copy submap1
            *merged += *transformed_submap2;   // append transformed submap2
            save_pcd_file("/tmp/loop_detection_results/", base + "_after_optim.pcd", merged);
            // TODO: test code
            save_pcd_file("/tmp/loop_detection_results/", base + "_after_optim_raw.pcd", output);
        }
        ///////////////////////////////////////////////////////////////////////////////////////////
    }

    std::deque<LoopCandidate> loop_candidates_;
    std::deque<LoopCandidate> successful_loop_candidates_;

    YamlLoadedConfig options_;
};
}   // namespace halo