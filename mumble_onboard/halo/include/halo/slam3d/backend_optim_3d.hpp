#pragma once

#include <halo/common/sensor_data_definitions.hpp>

// Keyframe: need id
// 2. remove_groud
// Build submap: can we use gaussian voxel instead?

namespace halo {
/**

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
}
