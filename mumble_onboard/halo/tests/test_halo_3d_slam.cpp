#include <gtest/gtest.h>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>

DEFINE_string(bag_path, "./data/ulhk/test2.txt", "path to rosbag");
DEFINE_string(yaml_config_path, "", "Path to yaml config");
DEFINE_int64(stopping_msg_index, 10000000, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_msg_index, 0, "start visualization from this index");

/**

// TODO: do we need EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
class KeyFrame{
    // can a static class func access private members of an input KeyFrame obj? TODO
    SE3 lidar_pose_
};

class IEKFLio{
    get_current_state();
    NavState queue;
    std::vector<NavStated> imu_states_;
};

// TODO: can save keyframes when the queue is full.
// Currently don't have disk loading / offloading, because we work w/ 1000 - 10000 frames. That's under 5G of gigs
// TODO: loading / offloading, using DMA. Also, we can copy frames -> {kframe_id: key_frame} in memory, when we are about to optimize

class HaloSLAM3DFrontend{
    add_cloud(cloud){
        s = iekf_lio.get_current_state();
        check_create_keyframe(s);
    }
    // Get keyframes, from current LIO
    get_keyframes() -> unique_ptr<vector<KeyFrame>>{
    }

    iekf_lio -> NavState queu
};


struct LoopCandidate{
    LoopCandidate(id1, id2, pose)
};

class HaloSLAMLoopDetection3D{
    void run(){
        DetectLoopCandidates(); // get loop_candiates_
        RemoveGround(clouds, 0.1);  // if (pt.z > z_min) { output->points.emplace_back(pt); }
        ComputeLoopCandidates();    // use
        SaveResults();  // save P of idx1, idx2 of keyframes
    }

    ComputeForCandidate(){
        // build submap
            // find some submap_idx_range, put them together
        // TODO: profile NDT and my own
        // pcl::NormalDistributionsTransform<PointType, PointType> ndt;

        for (auto& r : res) {
            ndt.setResolution(r);
        }
        // ?? TODO
        c.ndt_score_ = ndt.getTransformationProbability();

    }
    }
};

class HaloSLAM3DOptim{

    run(){
        kf_queue; loop_candidates
        if (!rtk_has_rot_ && stage_ == 1) {
            InitialAlign();
        }

        BuildProblem();  // 建立问题

        Solve();           // 带着RK求解一遍
        RemoveOutliers();  // 移除异常值
        Solve();           // 再求解一遍

        SaveResults();  // 保存结果

    }

    BuildProblem(){
        AddVertices();  // v is pose of each keyframes_ as the initial guess
        AddLidarEdges();    // add edge between every pair of keyframes. e is the keyframe guess
        AddLoopEdges(); // e is the loop candidate relative pose
    }  // 建立问题

    SaveResults(){
        v.second->write(fout);
        e->write(fout); // lidar edge and loop edge
    }

};

*/

TEST(HALOSLAM3DTest, test_halo_lidar_only_slam_3d) {
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
