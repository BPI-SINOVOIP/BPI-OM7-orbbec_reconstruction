#include "ransac_registration.hpp"
#include <vector>
#include <Eigen/Dense>

namespace reconstruction {
namespace {

// Load cloud, build downsampled representation, and compute FPFH features.
//
// Returns:
//   original cloud, downsampled cloud, FPFH feature descriptor for downsampled cloud.
std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::pipelines::registration::Feature>>
PreprocessPointCloud(const std::string& file_name, const double voxel_size) {
    auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
    if(!pcd) {
        return std::make_tuple(nullptr, nullptr, nullptr);
    }

    auto pcd_down = pcd->VoxelDownSample(voxel_size);
    if(!pcd_down) {
        return std::make_tuple(pcd, nullptr, nullptr);
    }

    pcd_down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(2.0 * voxel_size, 30));

    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *pcd_down,
        open3d::geometry::KDTreeSearchParamHybrid(5.0 * voxel_size, 100));

    return std::make_tuple(pcd, pcd_down, pcd_fpfh);
}

}  // namespace

bool RunRansacRegistrationFromFiles(
    const std::string& source_file,
    const std::string& target_file,
    const RansacRegistrationOptions& options,
    RansacRegistrationResult& result,
    std::string& error) {
    result = RansacRegistrationResult{};
    error.clear();

    // Validate options early to provide immediate and clear feedback.
    if(options.voxel_size <= 0.0) {
        error = "voxel_size must be > 0";
        return false;
    }

    if(options.distance_multiplier <= 0.0) {
        error = "distance_multiplier must be > 0";
        return false;
    }

    const std::string kMethodFeature = "feature_matching";
    const std::string kMethodCorres = "correspondence";
    if(options.method != kMethodFeature && options.method != kMethodCorres) {
        error = "method must be 'feature_matching' or 'correspondence'";
        return false;
    }

    std::shared_ptr<open3d::geometry::PointCloud> source;
    std::shared_ptr<open3d::geometry::PointCloud> source_down;
    std::shared_ptr<open3d::geometry::PointCloud> target;
    std::shared_ptr<open3d::geometry::PointCloud> target_down;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh;
    std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh;

    // Preprocess source/target independently: load, downsample, normals, FPFH.
    std::tie(source, source_down, source_fpfh) =
        PreprocessPointCloud(source_file, options.voxel_size);
    std::tie(target, target_down, target_fpfh) =
        PreprocessPointCloud(target_file, options.voxel_size);

    if(!source || !target || source->IsEmpty() || target->IsEmpty()) {
        error = "failed to load source/target cloud or cloud is empty";
        return false;
    }

    if(!source_down || !target_down || source_down->IsEmpty() || target_down->IsEmpty()) {
        error = "downsampled source/target cloud is empty";
        return false;
    }

    if(!source_fpfh || !target_fpfh) {
        error = "failed to compute FPFH feature";
        return false;
    }

    // Distance threshold scales with voxel size so behavior remains stable
    // when users change the global registration resolution.
    const double distance_threshold = options.voxel_size * options.distance_multiplier;

    // Common correspondence checkers used by both registration modes.
    std::vector<std::reference_wrapper<
            const open3d::pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;
    auto correspondence_checker_edge_length =
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance =
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
            distance_threshold);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    open3d::pipelines::registration::RegistrationResult registration_result;

    if(options.method == kMethodFeature) {
        // Feature-matching mode delegates correspondence search to Open3D.
        registration_result =
            open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
                *source_down,
                *target_down,
                *source_fpfh,
                *target_fpfh,
                options.mutual_filter,
                distance_threshold,
                open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
                3,
                correspondence_checker,
                open3d::pipelines::registration::RANSACConvergenceCriteria(
                    options.max_iterations,
                    options.confidence));
    }
    else {
        // Correspondence mode performs explicit nearest-neighbor matching in FPFH space.
        // This gives tighter control over correspondences and optional mutual filtering.
        int nPti = static_cast<int>(source_down->points_.size());
        int nPtj = static_cast<int>(target_down->points_.size());

        open3d::geometry::KDTreeFlann feature_tree_i(*source_fpfh);
        open3d::geometry::KDTreeFlann feature_tree_j(*target_fpfh);

        open3d::pipelines::registration::CorrespondenceSet corres_ji;
        corres_ji.reserve(nPtj);

        for(int j = 0; j < nPtj; j++) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);
            feature_tree_i.SearchKNN(Eigen::VectorXd(target_fpfh->data_.col(j)),
                                     1,
                                     corres_tmp,
                                     dist_tmp);
            int i = corres_tmp[0];
            corres_ji.push_back(Eigen::Vector2i(i, j));
        }

        if(options.mutual_filter) {
            // Keep only bidirectional correspondences for better robustness.
            open3d::pipelines::registration::CorrespondenceSet mutual_corres;
            mutual_corres.reserve(corres_ji.size());
            for(const auto& corres : corres_ji) {
                int j = corres(1);
                int j2i = corres(0);

                std::vector<int> corres_tmp(1);
                std::vector<double> dist_tmp(1);
                feature_tree_j.SearchKNN(Eigen::VectorXd(source_fpfh->data_.col(j2i)),
                                         1,
                                         corres_tmp,
                                         dist_tmp);
                int i2j = corres_tmp[0];
                if(i2j == j) {
                    mutual_corres.push_back(corres);
                }
            }

            registration_result =
                open3d::pipelines::registration::RegistrationRANSACBasedOnCorrespondence(
                    *source_down,
                    *target_down,
                    mutual_corres,
                    distance_threshold,
                    open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
                    3,
                    correspondence_checker,
                    open3d::pipelines::registration::RANSACConvergenceCriteria(
                        options.max_iterations,
                        options.confidence));
        }
        else {
            registration_result =
                open3d::pipelines::registration::RegistrationRANSACBasedOnCorrespondence(
                    *source_down,
                    *target_down,
                    corres_ji,
                    distance_threshold,
                    open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
                    3,
                    correspondence_checker,
                    open3d::pipelines::registration::RANSACConvergenceCriteria(
                        options.max_iterations,
                        options.confidence));
        }
    }

    result.valid = registration_result.transformation_.allFinite();
    result.fitness = registration_result.fitness_;
    result.inlier_rmse = registration_result.inlier_rmse_;
    result.transformation = registration_result.transformation_;
    result.source = source;
    result.target = target;
    result.source_down = source_down;
    result.target_down = target_down;

    if(!result.valid) {
        error = "registration transformation is not finite";
        return false;
    }

    return true;
}

void VisualizeRegistrationResult(const RansacRegistrationResult& result) {
    // Visualization helper used for debugging one registration pair.
    if(!result.source || !result.target) {
        return;
    }

    auto source_transformed_ptr = std::make_shared<open3d::geometry::PointCloud>();
    auto target_ptr = std::make_shared<open3d::geometry::PointCloud>();
    *source_transformed_ptr = *result.source;
    *target_ptr = *result.target;
    source_transformed_ptr->Transform(result.transformation);

    open3d::visualization::DrawGeometries(
        {source_transformed_ptr, target_ptr},
        "Registration result");
}

}  // namespace reconstruction
