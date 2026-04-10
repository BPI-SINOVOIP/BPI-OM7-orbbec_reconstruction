#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace reconstruction {

// User-tunable options for global RANSAC registration.
//
// method:
//   "feature_matching"  -> Open3D FPFH feature-matching RANSAC
//   "correspondence"    -> explicit nearest-neighbor correspondences in FPFH space
struct RansacRegistrationOptions {
    std::string method = "feature_matching";
    double voxel_size = 0.01;
    double distance_multiplier = 1.5;
    int max_iterations = 1000000;
    double confidence = 0.999;
    bool mutual_filter = false;
};

// Registration outputs used by merging and optional visualization.
// transformation maps source cloud into target cloud frame.
struct RansacRegistrationResult {
    bool valid = false;
    double fitness = 0.0;
    double inlier_rmse = 0.0;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    std::shared_ptr<open3d::geometry::PointCloud> source;
    std::shared_ptr<open3d::geometry::PointCloud> target;
    std::shared_ptr<open3d::geometry::PointCloud> source_down;
    std::shared_ptr<open3d::geometry::PointCloud> target_down;
};

// Register source cloud to target cloud using the selected RANSAC method.
// On success: returns true and fills result.
// On failure: returns false and writes a human-readable reason into error.
bool RunRansacRegistrationFromFiles(
    const std::string& source_file,
    const std::string& target_file,
    const RansacRegistrationOptions& options,
    RansacRegistrationResult& result,
    std::string& error);

// Utility visualization of one registration result.
void VisualizeRegistrationResult(const RansacRegistrationResult& result);

}  // namespace reconstruction
