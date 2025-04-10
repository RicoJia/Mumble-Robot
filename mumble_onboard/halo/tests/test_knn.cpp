// Currently, run this file in the mumble_onboard container because it has the
// G2O and stuff.
#include <gtest/gtest.h>

#include <halo/common/halo_io.hpp>                   // because this is added in cmake
#include <halo/common/math_utils.hpp>                // because this is added in cmake
#include <halo/common/sensor_data_definitions.hpp>   // because this is added in cmake
#include <halo/kd_tree.hpp>
#include <halo/nanoflann_kdtree.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/octo_tree.hpp>
#include <unordered_set>

constexpr const char *first_scan_path = "/home/mumble_robot/data/ch5/first.pcd";
constexpr const char *second_scan_path =
    "/home/mumble_robot/data/ch5/second.pcd";

double computeEuclideanDistance(const auto &p1, const auto &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;   // Remove if working in 2D.
    return dx * dx + dy * dy + dz * dz;
}

void evaluate_matches(
    const std::vector<halo::NNMatch> &test_matches,
    const std::vector<halo::NNMatch> &ground_truth_matches,
    size_t k,
    halo::PCLCloudXYZIPtr first,
    halo::PCLCloudXYZIPtr second,
    const bool &debug_print = true) {
    if (test_matches.empty() || ground_truth_matches.empty()) {
        std::cerr << "Empty match vectors!" << std::endl;
        return;
    }

    size_t num_queries    = ground_truth_matches.size();
    size_t true_positives = 0;
    size_t scaler         = 1;

    // Case 1: ground_truth has one match per query (brute-force)
    // and test_matches contains k results per query (e.g. from PCL or KD-tree)
    if (test_matches.size() == num_queries * k) {
        scaler = k;
        for (size_t q = 0; q < num_queries; ++q) {
            const auto &gt_match = ground_truth_matches[q];
            size_t base_idx      = q * k;
            // Look through the k results for this query
            for (size_t j = 0; j < k; ++j) {
                const auto &test_match = test_matches[base_idx + j];
                if (test_match.closest_pt_idx_in_other_cloud == gt_match.closest_pt_idx_in_other_cloud) {
                    true_positives++;
                    break;   // found the ground-truth match for this query
                }
            }
        }
    }
    // Case 2: both match sets have the same size.
    // This implies that ground_truth_matches is a KNN set (or both are 1-NN)
    // and we compare elementwise.
    else if (test_matches.size() == num_queries) {
        for (size_t q = 0; q < num_queries; q = q + k) {
            std::unordered_set<size_t> gt_set;
            for (size_t j = 0; j < k; ++j) {
                gt_set.insert(ground_truth_matches[q + j].closest_pt_idx_in_other_cloud);
            }
            for (size_t j = 0; j < k; ++j) {
                if (gt_set.find(test_matches[q + j].closest_pt_idx_in_other_cloud) != gt_set.end()) {
                    true_positives++;
                } else {
                    if (debug_print) {
                        auto second_idx_test = test_matches[q + j].closest_pt_idx_in_other_cloud;
                        auto second_idx_pcl  = ground_truth_matches[q + j].closest_pt_idx_in_other_cloud;
                        auto test_pt         = second->points.at(second_idx_test);
                        auto pcl_pt          = second->points.at(second_idx_pcl);
                        auto first_pt        = first->points.at(test_matches[q + j].idx_in_this_cloud);
                        auto test_dist       = computeEuclideanDistance(first_pt, test_pt);
                        auto pcl_dist        = computeEuclideanDistance(first_pt, pcl_pt);
                        std::cout << "test pt dist: " << test_dist << ", pcl dist: " << pcl_dist << std::endl;
                        std::cout << "couldn't find test match " << test_matches[q + j].closest_pt_idx_in_other_cloud << std::endl;
                    }
                }
            }
        }
        scaler = 1;
    } else {
        if (debug_print) {
            std::cout << "test match size differ from that of the ground_truth" << std::endl;
        }
        // matches may not have the same size as the test
        for (const auto &test_match : test_matches) {
            size_t id_gt = test_match.idx_in_this_cloud;
            if (ground_truth_matches.at(id_gt).closest_pt_idx_in_other_cloud == test_match.closest_pt_idx_in_other_cloud) {
                true_positives++;
            }
        }
    }

    // In the brute-force (1-NN) case with scaling,
    // test_matches.size()/scaler gives us the number of queries.
    float precision = static_cast<float>(true_positives) / (static_cast<float>(test_matches.size()) / scaler);
    float recall    = static_cast<float>(true_positives) / static_cast<float>(ground_truth_matches.size());
    float f1_score  = (precision + recall > 0.f) ? 2.f * (precision * recall) / (precision + recall) : 0.f;

    std::cout << "\nMatch Evaluation Results:" << std::endl;
    std::cout << "Precision: " << precision * 100 << "%" << std::endl;
    std::cout << "Recall: " << recall * 100 << "%" << std::endl;
    std::cout << "True Positives: " << true_positives << std::endl;
    std::cout << "F1 Score: " << f1_score * 100 << "%" << std::endl;
}

/**
 * Performance: 19k points,
    resolution = 0.1m, 3ms (NEARBY 18), recall: 79.0%, precision: 95.8%
    resolution = 0.5m, 4ms (NEARBY 18), recall: 95.6%, precision: 98.1%
    resolution = 2.0m, 30ms (NEARBY 18), recall: 99.0%, precision: 99.4%
 */
TEST(TestKNN, test_grid_method) {
    halo::PCLCloudXYZIPtr first;
    halo::PCLCloudXYZIPtr second;
    halo::PCLCloudXYZIPtr cloud{new halo::PCLCloudXYZI};

    first.reset(new halo::PCLCloudXYZI);
    second.reset(new halo::PCLCloudXYZI);
    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);

    halo::PCLCloudXYZIPtr test_cloud = second;
    std::vector<halo::NNMatch> ground_truth_matches =
        halo::brute_force_nn(first, test_cloud, true);
    std::vector<halo::NNMatch> matches;
    {
        std::cout << "=====================Case 0: Grid Method k = 1=====================" << std::endl;
        float resolution = 5.0;   // 5 pixels per meter
        halo::RAIITimer timer;
        halo::NearestNeighborGrid<3, halo::NeighborCount::NEARBY18> nn_grid(resolution);
        nn_grid.set_pointcloud(first);
        matches = nn_grid.get_closest_point(test_cloud);
    }
    evaluate_matches(matches, ground_truth_matches, 1, first, second, false);
}

/**
 * Performance: 19k points,
    - 100% accuracy, 7ms
    - 100% accuracy, 6ms, k = 5
    - PCL: 0.18ms, k=5
 */
TEST(TestKNN, test_kd_tree) {
    halo::PCLCloudXYZIPtr first;
    halo::PCLCloudXYZIPtr second;
    halo::PCLCloudXYZIPtr cloud{new halo::PCLCloudXYZI};

    first.reset(new halo::PCLCloudXYZI);
    second.reset(new halo::PCLCloudXYZI);
    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);
    // halo::downsample_point_cloud(first, 0.05f);
    // halo::downsample_point_cloud(second, 0.05f);

    halo::PCLCloudXYZIPtr test_cloud = second;
    std::vector<halo::NNMatch> ground_truth_matches =
        halo::brute_force_nn(first, test_cloud, true);

    std::vector<halo::NNMatch> matches;
    {
        std::cout << "=====================Case 1: k = 1=====================" << std::endl;
        halo::RAIITimer timer;
        halo::KDTree kd_tree(first, 1.0);
        size_t k = 1;
        // 4.5ms
        kd_tree.search_tree_multi_threaded(test_cloud, matches, k);
        EXPECT_EQ(matches.size(), second->points.size() * k);
    }
    evaluate_matches(matches, ground_truth_matches, 1, first, second);

    size_t k = 5;
    {
        std::cout << "=====================Case 2: k = 5=====================" << std::endl;
        halo::RAIITimer timer;
        halo::KDTree kd_tree(first, 1.0);
        // 4.5ms
        kd_tree.search_tree_multi_threaded(test_cloud, matches, k);
    }
    evaluate_matches(matches, ground_truth_matches, 5, first, second);

    std::vector<halo::NNMatch> pcl_matches;
    {
        std::cout << "=====================Case 3: k = 5, PCL=====================" << std::endl;
        std::vector<std::vector<int>> result_index;
        pcl::search::KdTree<halo::PCLPointXYZI> kdtree_pcl;

        // 对比PCL: 0.18s
        halo::RAIITimer timer;
        kdtree_pcl.setInputCloud(first);
        std::vector<int> search_indices(second->size());
        for (size_t i = 0; i < second->points.size(); i++) {
            search_indices[i] = i;
        }

        std::vector<std::vector<float>> result_distance;
        kdtree_pcl.nearestKSearch(*second, search_indices, 5, result_index, result_distance);
        for (size_t i = 0; i < second->points.size(); ++i) {   // Iterate over all query points
            for (size_t j = 0; j < std::min(k, result_index[i].size()); ++j) {
                size_t m = result_index[i][j];                   // Index of nearest neighbor in 'first'
                pcl_matches.emplace_back(halo::NNMatch{i, m});   // Store the match
            }
        }
    }
    evaluate_matches(matches, pcl_matches, 5, first, second);

    {
        std::cout << "=====================" << std::endl;
    }
}

// Test for 2D bounding box creation.
TEST(OctoTreeNodeTest, BoundingBox2D) {
    using Node2D = halo::OctoTreeNode<2>;
    using Vec2   = Eigen::Matrix<float, 2, 1>;

    // Create a vector of 2D points.
    std::vector<Vec2> points;
    points.push_back((Vec2() << 1.0f, 2.0f).finished());
    points.push_back((Vec2() << 3.0f, 1.0f).finished());
    points.push_back((Vec2() << 2.0f, 4.0f).finished());
    points.push_back((Vec2() << -1.0f, 0.5f).finished());

    // All indices to use.
    std::vector<size_t> indices = {0, 1, 2, 3};

    Node2D node;
    node.create_bounding_box(indices, points);

    // Expected lower: minimum x and y values.
    Vec2 expectedLower, expectedUpper;
    expectedLower << -1.0f, 0.5f;
    expectedUpper << 3.0f, 4.0f;

    ASSERT_TRUE(node.box_ptr_ != nullptr);
    EXPECT_FLOAT_EQ(node.box_ptr_->lower(0), expectedLower(0));
    EXPECT_FLOAT_EQ(node.box_ptr_->lower(1), expectedLower(1));
    EXPECT_FLOAT_EQ(node.box_ptr_->upper(0), expectedUpper(0));
    EXPECT_FLOAT_EQ(node.box_ptr_->upper(1), expectedUpper(1));
}

TEST(OctoTreeNodeTest, BoundingBox3D) {
    using Node3D = halo::OctoTreeNode<3>;
    using Vec3   = Eigen::Matrix<float, 3, 1>;

    // Create a vector of 3D points.
    std::vector<Vec3> points;
    points.push_back((Vec3() << 1.0f, 2.0f, 3.0f).finished());
    points.push_back((Vec3() << 4.0f, 0.0f, 2.0f).finished());
    points.push_back((Vec3() << -1.0f, 5.0f, 0.0f).finished());
    points.push_back((Vec3() << 2.0f, 1.0f, 4.0f).finished());

    // All indices to use.
    std::vector<size_t> indices = {0, 1, 2, 3};

    Node3D node;
    node.create_bounding_box(indices, points);

    // Expected lower: minimum values for x, y, and z.
    Vec3 expectedLower, expectedUpper;
    expectedLower << -1.0f, 0.0f, 0.0f;
    expectedUpper << 4.0f, 5.0f, 4.0f;

    ASSERT_TRUE(node.box_ptr_ != nullptr);
    EXPECT_FLOAT_EQ(node.box_ptr_->lower(0), expectedLower(0));
    EXPECT_FLOAT_EQ(node.box_ptr_->lower(1), expectedLower(1));
    EXPECT_FLOAT_EQ(node.box_ptr_->lower(2), expectedLower(2));
    EXPECT_FLOAT_EQ(node.box_ptr_->upper(0), expectedUpper(0));
    EXPECT_FLOAT_EQ(node.box_ptr_->upper(1), expectedUpper(1));
    EXPECT_FLOAT_EQ(node.box_ptr_->upper(2), expectedUpper(2));
}
/**
 * Performance:
 * - 100%, 18ms
 * - 21ms, k=5
 * - 20ms, pcl
 */
TEST(TestKNN, test_octo_tree) {
    halo::PCLCloudXYZIPtr first;
    halo::PCLCloudXYZIPtr second;
    halo::PCLCloudXYZIPtr cloud{new halo::PCLCloudXYZI};

    first.reset(new halo::PCLCloudXYZI);
    second.reset(new halo::PCLCloudXYZI);
    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);
    // halo::downsample_point_cloud(first, 0.05f);
    // halo::downsample_point_cloud(second, 0.05f);

    halo::PCLCloudXYZIPtr test_cloud = second;
    std::vector<halo::NNMatch> matches;
    std::vector<halo::NNMatch> ground_truth_matches =
        halo::brute_force_nn(first, test_cloud, true);

    {
        std::cout << "=====================Octo Tree Case 0: Octo tree building=====================" << std::endl;
        halo::RAIITimer timer;
        halo::OctoTree octo_tree(first, 1.0);

        EXPECT_EQ(octo_tree.get_non_leaf_num(), first->points.size());
        std::cout << "size: " << first->points.size() << std::endl;
    }

    {
        std::cout << "=====================Octo Tree Case 1: k = 1=====================" << std::endl;
        halo::RAIITimer timer;
        halo::OctoTree octo_tree(first, 1.0);
        size_t k = 1;
        // 5ms
        octo_tree.search_tree_multi_threaded(test_cloud, matches, k);
        EXPECT_EQ(matches.size(), test_cloud->points.size() * k);
    }
    evaluate_matches(matches, ground_truth_matches, 1, first, second);

    size_t k = 5;
    {
        std::cout << "=====================Octo Tree Case 2: k = 5=====================" << std::endl;
        halo::RAIITimer timer;
        halo::OctoTree octo_tree(first, 1.0);
        // 4.5ms
        octo_tree.search_tree_multi_threaded(test_cloud, matches, k);
    }
    evaluate_matches(matches, ground_truth_matches, 5, first, second);

    std::vector<halo::NNMatch> pcl_matches;
    {
        std::cout << "=====================Octo Tree Case 3: k = 5, PCL=====================" << std::endl;
        std::vector<std::vector<int>> result_index;
        pcl::search::KdTree<halo::PCLPointXYZI> kdtree_pcl;

        // 对比PCL: 0.18s
        halo::RAIITimer timer;
        kdtree_pcl.setInputCloud(first);
        std::vector<int> search_indices(test_cloud->size());
        for (size_t i = 0; i < test_cloud->points.size(); i++) {
            search_indices[i] = i;
        }

        std::vector<std::vector<float>> result_distance;
        kdtree_pcl.nearestKSearch(*test_cloud, search_indices, 5, result_index, result_distance);
        for (size_t i = 0; i < test_cloud->points.size(); ++i) {   // Iterate over all query points
            for (size_t j = 0; j < std::min(k, result_index[i].size()); ++j) {
                size_t m = result_index[i][j];                   // Index of nearest neighbor in 'first'
                pcl_matches.emplace_back(halo::NNMatch{i, m});   // Store the match
            }
        }
    }
    evaluate_matches(matches, pcl_matches, 5, first, test_cloud);
}

/**
 * Performance:
 * - k = 1, 100%, 4ms
 * - k = 5, 100%, 4ms
 */
TEST(TestKNN, test_nanoflann_kdtree) {
    // Load point clouds from files.
    halo::PCLCloudXYZIPtr first(new halo::PCLCloudXYZI);
    halo::PCLCloudXYZIPtr second(new halo::PCLCloudXYZI);

    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);

    // Use the second cloud as the query set.
    halo::PCLCloudXYZIPtr test_cloud = second;
    std::vector<halo::NNMatch> matches;
    std::vector<halo::NNMatch> ground_truth_matches =
        halo::brute_force_nn(first, test_cloud, true);

    {
        std::cout << "=====================NanoFlann Case0: k = 1 for Nanoflann KD Tree=====================" << std::endl;
        halo::RAIITimer timer;
        halo::NanoflannPointCloudAdaptor<halo::PCLPointXYZI> adaptor(*first);
        halo::NanoFlannKDTree<halo::PCLPointXYZI, 3> nano_tree(adaptor,
                                                               nanoflann::KDTreeSingleIndexAdaptorParams(10));
        size_t k = 1;
        nano_tree.search_tree_multi_threaded(test_cloud, matches, k);
        EXPECT_EQ(matches.size(), second->points.size() * k);
    }
    evaluate_matches(matches, ground_truth_matches, 1, first, second);
}