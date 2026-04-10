#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <chrono>

#include <open3d/Open3D.h>

#include "ransac_registration.hpp"


namespace {

// This executable performs pairwise global registration across indexed point-cloud files,
// merges them into one model, applies cleanup/downsampling, and writes registered_cloud.ply.
//
// Expected input naming convention:
//   <input_data_dir>/<file_prefix><index><file_ext>
// Example:
//   /path/to/filtered_data/data_0.ply
//   /path/to/filtered_data/data_1.ply

// Runtime configuration loaded from command-line arguments.
struct Config {
    // Input sequence location and naming policy.
    std::string input_data_dir = "/home/pi/repo/orbbec_reconstruction/data/object_data/filtered_data";
    std::string result_dir = "/home/pi/repo/orbbec_reconstruction/data/object_data/result";
    std::string file_prefix = "data_";
    std::string file_ext = ".ply";
    int start_index = 0;
    int end_index = -1;

    // RANSAC registration options used for every adjacent pair.
    reconstruction::RansacRegistrationOptions reg;

    // Merge/memory safety controls for accumulated cloud size.
    double merge_voxel_size = 0.0;
    int merge_max_points = 1500000;
    double merge_guard_voxel_start = 0.01;

    // Final statistical outlier removal parameters.
    int outlier_nb_neighbors = 20;
    double outlier_std_ratio = 2.0;

    // Enable final interactive Open3D visualization.
    bool visualize = true;
};

std::string Trim(const std::string& s) {
    size_t b = 0;
    while(b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    size_t e = s.size();
    while(e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool ParseBool(const std::string& s, bool& out) {
    const std::string v = ToLower(Trim(s));
    if(v == "1" || v == "true" || v == "yes" || v == "on") {
        out = true;
        return true;
    }
    if(v == "0" || v == "false" || v == "no" || v == "off") {
        out = false;
        return true;
    }
    return false;
}

void PrintUsage(const char* prog) {
    std::cout
        << "Usage:\n"
        << "  " << prog << " [--key=value ...]\n\n"
        << "Common options:\n"
    << "  --input_data_dir=PATH          Directory containing input point clouds\n"
    << "  --result_dir=PATH              Directory for output registered_cloud.ply\n"
    << "  --file_prefix=STR              Input filename prefix (default: data_)\n"
    << "  --file_ext=EXT                 Input filename extension (default: .ply)\n"
    << "  --start_index=N                Start index (default: 0)\n"
    << "  --end_index=N                  End index, -1 means auto-stop (default: -1)\n"
    << "  --method=feature_matching|fgr  RANSAC method\n"
    << "  --voxel_size=FLOAT             RANSAC voxel size (default: 0.01)\n"
    << "  --distance_multiplier=FLOAT    RANSAC distance multiplier (default: 1.5)\n"
    << "  --max_iterations=N             RANSAC max iterations (default: 1000000)\n"
    << "  --confidence=FLOAT             RANSAC confidence (default: 0.999)\n"
    << "  --mutual_filter=0|1            Enable mutual filtering (default: 0)\n"
    << "  --merge_voxel_size=FLOAT       Optional merge downsample voxel (default: 0.0)\n"
    << "  --merge_max_points=N           Merge point budget (default: 1500000)\n"
    << "  --merge_guard_voxel_start=FLOAT Merge guard start voxel (default: 0.01)\n"
    << "  --outlier_nb_neighbors=N       Outlier filter neighbors (default: 20)\n"
    << "  --outlier_std_ratio=FLOAT      Outlier filter std-ratio (default: 2.0)\n"
    << "  --visualize=0|1                Show final viewer (default: 1)\n"
    << "  -h, --help                     Show this help message\n";
}

bool ParseArgs(int argc, char* argv[], Config& cfg, std::string& error) {
    // Parse all arguments in --key=value format so invocation stays script-friendly.
    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "-h" || arg == "--help") {
            return false;
        }
        if(arg.rfind("--", 0) != 0) {
            error = "invalid argument format: " + arg + " (expect --key=value)";
            return false;
        }
        auto eq = arg.find('=');
        if(eq == std::string::npos || eq <= 2) {
            error = "invalid argument format: " + arg + " (expect --key=value)";
            return false;
        }

        std::string key = arg.substr(2, eq - 2);
        std::string value = arg.substr(eq + 1);
        try {
            if(key == "input_data_dir") cfg.input_data_dir = value;
            else if(key == "result_dir") cfg.result_dir = value;
            else if(key == "file_prefix") cfg.file_prefix = value;
            else if(key == "file_ext") cfg.file_ext = value;
            else if(key == "start_index") cfg.start_index = std::stoi(value);
            else if(key == "end_index") cfg.end_index = std::stoi(value);
            else if(key == "method") cfg.reg.method = value;
            else if(key == "voxel_size") cfg.reg.voxel_size = std::stod(value);
            else if(key == "distance_multiplier") cfg.reg.distance_multiplier = std::stod(value);
            else if(key == "max_iterations") cfg.reg.max_iterations = std::stoi(value);
            else if(key == "confidence") cfg.reg.confidence = std::stod(value);
            else if(key == "mutual_filter") {
                bool b = false;
                if(!ParseBool(value, b)) {
                    throw std::runtime_error("invalid bool");
                }
                cfg.reg.mutual_filter = b;
            }
            else if(key == "merge_voxel_size") cfg.merge_voxel_size = std::stod(value);
            else if(key == "merge_max_points") cfg.merge_max_points = std::stoi(value);
            else if(key == "merge_guard_voxel_start") cfg.merge_guard_voxel_start = std::stod(value);
            else if(key == "outlier_nb_neighbors") cfg.outlier_nb_neighbors = std::stoi(value);
            else if(key == "outlier_std_ratio") cfg.outlier_std_ratio = std::stod(value);
            else if(key == "visualize") {
                bool b = false;
                if(!ParseBool(value, b)) {
                    throw std::runtime_error("invalid bool");
                }
                cfg.visualize = b;
            }
            else {
                error = "unknown key: " + key;
                return false;
            }
        }
        catch(const std::exception&) {
            error = "invalid value for key: " + key;
            return false;
        }
    }

    if(cfg.start_index < 0) {
        error = "start_index must be >= 0";
        return false;
    }
    if(cfg.input_data_dir.empty() || cfg.result_dir.empty()) {
        error = "input_data_dir and result_dir must not be empty";
        return false;
    }
    if(cfg.end_index >= 0 && cfg.end_index < cfg.start_index) {
        error = "end_index must be >= start_index, or use -1 for auto-stop";
        return false;
    }
    if(cfg.merge_max_points <= 10000) {
        error = "merge_max_points must be > 10000";
        return false;
    }
    if(cfg.merge_guard_voxel_start <= 0.0) {
        error = "merge_guard_voxel_start must be > 0";
        return false;
    }
    if(cfg.outlier_nb_neighbors <= 0 || cfg.outlier_std_ratio <= 0.0) {
        error = "outlier params must be > 0";
        return false;
    }

    return true;
}

bool FileExists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool IsDirectory(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool EnsureDirectory(const std::string& path) {
    if(IsDirectory(path)) {
        return true;
    }
    if(mkdir(path.c_str(), 0755) == 0) {
        return true;
    }
    return errno == EEXIST && IsDirectory(path);
}

std::shared_ptr<open3d::geometry::PointCloud> KeepFinitePoints(
    const std::shared_ptr<open3d::geometry::PointCloud>& cloud) {
    // Remove NaN/Inf points so downstream Open3D ops remain stable.
    auto out = std::make_shared<open3d::geometry::PointCloud>();
    if(!cloud) {
        return out;
    }

    const bool has_colors = cloud->colors_.size() == cloud->points_.size();
    out->points_.reserve(cloud->points_.size());
    if(has_colors) {
        out->colors_.reserve(cloud->colors_.size());
    }

    for(size_t i = 0; i < cloud->points_.size(); ++i) {
        const auto& p = cloud->points_[i];
        if(!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
            continue;
        }
        out->points_.push_back(p);
        if(has_colors) {
            out->colors_.push_back(cloud->colors_[i]);
        }
    }
    return out;
}

std::string JoinPath(const std::string& a, const std::string& b) {
    if(a.empty()) {
        return b;
    }
    if(a.back() == '/') {
        return a + b;
    }
    return a + "/" + b;
}

std::string BuildIndexedFilePath(const Config& cfg, int index) {
    return JoinPath(cfg.input_data_dir, cfg.file_prefix + std::to_string(index) + cfg.file_ext);
}

void CompactCloudToPointBudget(
    std::shared_ptr<open3d::geometry::PointCloud>& cloud,
    int max_points,
    double start_voxel,
    const std::string& tag) {
    if(!cloud || cloud->IsEmpty() || max_points <= 0) {
        return;
    }
    if(static_cast<int>(cloud->points_.size()) <= max_points) {
        return;
    }

    double voxel = std::max(start_voxel, 1e-6);
    while(static_cast<int>(cloud->points_.size()) > max_points) {
        auto down = cloud->VoxelDownSample(voxel);
        if(!down || down->IsEmpty() || down->points_.size() >= cloud->points_.size()) {
            break;
        }
        cloud = down;
        voxel *= 1.5;
    }

    std::cout << "[" << tag << "] points after budget compact: "
              << cloud->points_.size() << std::endl;
}

std::vector<std::string> CollectInputFiles(const Config& cfg) {
    // Build an ordered file list either by fixed range or auto-stop mode.
    //
    // end_index >= 0: explicit bounded range [start_index, end_index].
    // end_index < 0 : auto-stop on first missing file after at least one success.
    std::vector<std::string> files;
    if(cfg.end_index >= 0) {
        for(int i = cfg.start_index; i <= cfg.end_index; ++i) {
            auto path = BuildIndexedFilePath(cfg, i);
            if(FileExists(path)) {
                files.push_back(path);
            }
            else {
                std::cerr << "[Warn] missing file: " << path << std::endl;
            }
        }
        return files;
    }

    for(int i = cfg.start_index;; ++i) {
        auto path = BuildIndexedFilePath(cfg, i);
        if(!FileExists(path)) {
            if(i == cfg.start_index) {
                return files;
            }
            break;
        }
        files.push_back(path);
    }
    return files;
}

}  // namespace

int main(int argc, char* argv[]) {
    auto total_start = std::chrono::steady_clock::now();

    Config cfg;
    std::string error;
    if(!ParseArgs(argc, argv, cfg, error)) {
        if(!error.empty()) {
            std::cerr << "Failed to parse args: " << error << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
        PrintUsage(argv[0]);
        return 0;
    }

    auto files = CollectInputFiles(cfg);
    if(files.size() < 2) {
        // Registration is pairwise, so at least two clouds are required.
        std::cerr << "Need at least 2 point-cloud files, found: " << files.size() << std::endl;
        return 1;
    }

    std::cout << "[Args] input_data_dir=" << cfg.input_data_dir << std::endl;
    std::cout << "[Args] result_dir=" << cfg.result_dir << std::endl;
    std::cout << "[Args] input files: " << files.size() << std::endl;
    for(size_t i = 0; i < files.size(); ++i) {
        std::cout << "  [" << i << "] " << files[i] << std::endl;
    }
    std::cout << "[Args] method=" << cfg.reg.method
              << ", voxel_size=" << cfg.reg.voxel_size
              << ", distance_multiplier=" << cfg.reg.distance_multiplier << std::endl;
    std::cout << "[Args] merge_max_points=" << cfg.merge_max_points
              << ", merge_guard_voxel_start=" << cfg.merge_guard_voxel_start << std::endl;

    // Stage 1: pairwise RANSAC registration and global merge.
    //
    // For every i:
    //   T(i -> i-1) from RANSAC, then chain into base frame:
    //   T(i -> base) = T(i-1 -> base) * T(i -> i-1)
    auto merged = std::make_shared<open3d::geometry::PointCloud>();

    auto base_cloud = open3d::io::CreatePointCloudFromFile(files[0]);
    if(!base_cloud || base_cloud->IsEmpty()) {
        std::cerr << "Failed to load base cloud: " << files[0] << std::endl;
        return 1;
    }
    *merged += *base_cloud;
    base_cloud.reset();

    Eigen::Matrix4d prev_pose_to_base = Eigen::Matrix4d::Identity();

    
    for(size_t i = 1; i < files.size(); ++i) {
        auto file_start = std::chrono::steady_clock::now();
        reconstruction::RansacRegistrationResult reg_result;
        std::string reg_error;
        const bool ok = reconstruction::RunRansacRegistrationFromFiles(
            files[i],
            files[i - 1],
            cfg.reg,
            reg_result,
            reg_error);
        if(!ok) {
            std::cerr << "[Reg " << i << "] failed: " << reg_error << std::endl;
            return 1;
        }

        std::cout << "[Reg " << i << "] fitness=" << reg_result.fitness
                  << ", rmse=" << reg_result.inlier_rmse << std::endl;

        Eigen::Matrix4d curr_pose_to_base = prev_pose_to_base * reg_result.transformation;
        prev_pose_to_base = curr_pose_to_base;

        if(!reg_result.source || reg_result.source->IsEmpty()) {
            std::cerr << "[Reg " << i << "] source cloud is empty" << std::endl;
            return 1;
        }

        auto transformed = std::make_shared<open3d::geometry::PointCloud>(*reg_result.source);
        transformed->Transform(curr_pose_to_base);
        *merged += *transformed;

        transformed.reset();
        reg_result.source.reset();
        reg_result.target.reset();
        reg_result.source_down.reset();
        reg_result.target_down.reset();

        if(static_cast<int>(merged->points_.size()) > cfg.merge_max_points) {
            CompactCloudToPointBudget(
                merged,
                cfg.merge_max_points,
                cfg.merge_guard_voxel_start,
                "MergeGuard");
        }
        auto file_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> file_elapsed = file_end - file_start;
        std::cout << "[Time Consuming " << file_elapsed.count() << "s]" << std::endl;
    }

    // Stage 2: optional pre-clean downsampling to reduce memory usage.
    std::cout << "[Merge] points before optional downsample: " << merged->points_.size() << std::endl;
    if(cfg.merge_voxel_size > 0.0) {
        auto down = merged->VoxelDownSample(cfg.merge_voxel_size);
        if(down && !down->IsEmpty()) {
            merged = down;
            std::cout << "[Merge] points after voxel downsample: " << merged->points_.size() << std::endl;
        }
    }

    merged = KeepFinitePoints(merged);
    if(!merged || merged->IsEmpty()) {
        std::cerr << "Merged cloud is empty after finite-point filtering" << std::endl;
        return 1;
    }
    merged->RemoveDuplicatedPoints();

    // Stage 3: denoise and keep a manageable point budget for output.
    //
    // This stage intentionally uses a fixed output budget to avoid generating
    // excessively large files that are slow to load and visualize.
    auto outlier_tuple = merged->RemoveStatisticalOutliers(
        cfg.outlier_nb_neighbors,
        cfg.outlier_std_ratio);
    auto cleaned = std::get<0>(outlier_tuple);
    if(cleaned && !cleaned->IsEmpty()) {
        merged = cleaned;
    }

    const int output_max_points = 800000;
    double output_voxel = 0.003;
    while(static_cast<int>(merged->points_.size()) > output_max_points) {
        auto down = merged->VoxelDownSample(output_voxel);
        if(!down || down->IsEmpty() || down->points_.size() >= merged->points_.size()) {
            break;
        }
        merged = down;
        output_voxel *= 1.5;
    }

    std::cout << "[FinalPrep] points after cleanup: " << merged->points_.size() << std::endl;

    // Stage 4: persist merged cloud and optionally visualize it.
    if(!EnsureDirectory(cfg.result_dir)) {
        std::cerr << "Failed to ensure result directory: " << cfg.result_dir
                  << " (" << std::strerror(errno) << ")" << std::endl;
        return 1;
    }

    const std::string output_cloud_path = JoinPath(cfg.result_dir, "registered_cloud.ply");

    if(!open3d::io::WritePointCloud(output_cloud_path, *merged)) {
        std::cerr << "Failed to save merged cloud: " << output_cloud_path << std::endl;
        return 1;
    }
    std::cout << "[Output] merged cloud saved: " << output_cloud_path << std::endl;

    auto total_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> total_elapsed = total_end - total_start;
    std::cout << "[Total Time] " << total_elapsed.count() << "s" << std::endl;

    if(cfg.visualize) {
        open3d::visualization::DrawGeometries({merged}, "Merged Point Cloud Viewer");
    }

    return 0;
}