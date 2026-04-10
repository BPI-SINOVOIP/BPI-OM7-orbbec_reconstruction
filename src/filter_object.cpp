#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <open3d/Open3D.h>

namespace fs = std::filesystem;

namespace {

// Batch object extraction pipeline:
// 1) Enumerate files from input directory.
// 2) For each cloud, keep near-front depth slab.
// 3) Optionally keep only points near a target RGB color.
// 4) Optionally run DBSCAN and keep the largest cluster.
// 5) Save output cloud with original filename into output_dir.

struct Config {
    // Depth slab width measured from the closest observed point in each cloud.
    double depth_threshold = 0.5;

    // Optional color gating in normalized RGB space [0, 1].
    bool use_color_filter = false;
    double target_r = 0.0;
    double target_g = 0.0;
    double target_b = 0.0;
    double color_threshold = 0.2;
    bool use_clustering = false;
    double eps = 0.05;
    int min_points = 10;

    // Number of worker threads used for file-level parallelism.
    unsigned int threads = std::max(1u, std::thread::hardware_concurrency() / 2);

    // Input/output file scan policy.
    std::string input_dir = "data/object_data/raw_data";
    std::string output_dir = "data/object_data/tmp";
    std::string file_extension = ".ply";
};

struct BatchStats {
    int total = 0;
    int success = 0;
    int failed = 0;
};

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

void PrintUsage(const char* prog_name) {
    std::cout
        << "Usage:\n"
        << "  " << prog_name << " [options]\n\n"
        << "Description:\n"
        << "  Extract object points from all point-cloud files in a directory using\n"
        << "  front-depth cropping, optional color filtering, and optional largest-cluster selection.\n\n"
        << "Options:\n"
        << "  --depth-threshold <meters>         Keep points in [min_z, min_z + threshold] (default: 0.5)\n"
        << "  --color <R G B threshold>          Enable color filter with RGB in [0,255] and distance threshold\n"
        << "  --cluster <eps min_points>         Enable DBSCAN and keep largest cluster\n"
        << "  --input-dir <path>                 Input directory (default: data/object_data/raw_data)\n"
        << "  --output-dir <path>                Output directory (default: data/object_data/tmp)\n"
        << "  --ext <extension>                  File extension filter (default: .ply)\n"
        << "  --threads <n>                      Number of worker threads (default: half of CPU cores)\n"
        << "  -h, --help                         Show this help message\n\n"
        << "Examples:\n"
        << "  " << prog_name << " --depth-threshold 1.0\n"
        << "  " << prog_name << " --color 200 100 50 0.2 --cluster 0.05 10 --threads 4\n";
}

Config ParseArguments(int argc, char* argv[]) {
    // Parse GNU-style options that consume the following argument(s).
    Config config;

    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if(arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            std::exit(0);
        } else if(arg == "--depth-threshold" && i + 1 < argc) {
            config.depth_threshold = std::stod(argv[++i]);
        } else if(arg == "--color" && i + 4 < argc) {
            const int r = std::stoi(argv[++i]);
            const int g = std::stoi(argv[++i]);
            const int b = std::stoi(argv[++i]);
            config.color_threshold = std::stod(argv[++i]);
            config.target_r = static_cast<double>(r) / 255.0;
            config.target_g = static_cast<double>(g) / 255.0;
            config.target_b = static_cast<double>(b) / 255.0;
            config.use_color_filter = true;
        } else if(arg == "--cluster" && i + 2 < argc) {
            config.eps = std::stod(argv[++i]);
            config.min_points = std::stoi(argv[++i]);
            config.use_clustering = true;
        } else if(arg == "--input-dir" && i + 1 < argc) {
            config.input_dir = argv[++i];
        } else if(arg == "--output-dir" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if(arg == "--ext" && i + 1 < argc) {
            config.file_extension = argv[++i];
        } else if(arg == "--threads" && i + 1 < argc) {
            config.threads = static_cast<unsigned int>(std::max(1, std::stoi(argv[++i])));
        } else {
            std::cerr << "Unknown option or missing value: " << arg << std::endl;
            PrintUsage(argv[0]);
            std::exit(1);
        }
    }

    if(config.depth_threshold <= 0.0) {
        std::cerr << "depth-threshold must be > 0" << std::endl;
        std::exit(1);
    }
    if(config.use_color_filter && config.color_threshold < 0.0) {
        std::cerr << "color threshold must be >= 0" << std::endl;
        std::exit(1);
    }
    if(config.use_clustering && (config.eps <= 0.0 || config.min_points <= 0)) {
        std::cerr << "cluster parameters must satisfy eps > 0 and min_points > 0" << std::endl;
        std::exit(1);
    }

    return config;
}

bool HasExtensionMatch(const fs::path& file_path, const std::string& target_ext) {
    return ToLower(file_path.extension().string()) == ToLower(target_ext);
}

std::shared_ptr<open3d::geometry::PointCloud> ProcessPointCloud(
    const std::shared_ptr<open3d::geometry::PointCloud>& cloud,
    const Config& config) {
    // Per-file extraction routine that applies depth/color/cluster filters.
    if(!cloud || cloud->points_.empty()) {
        return nullptr;
    }

    // Use closest z value as dynamic front reference for this frame.
    double min_z = std::numeric_limits<double>::max();
    for(const auto& point : cloud->points_) {
        if(point.z() < min_z) {
            min_z = point.z();
        }
    }

    auto extracted_cloud = std::make_shared<open3d::geometry::PointCloud>();
    const bool has_colors = !cloud->colors_.empty() && cloud->colors_.size() == cloud->points_.size();
    const bool has_normals = !cloud->normals_.empty() && cloud->normals_.size() == cloud->points_.size();

    for(size_t i = 0; i < cloud->points_.size(); ++i) {
        const auto& point = cloud->points_[i];
        if(point.z() < min_z || point.z() > min_z + config.depth_threshold) {
            continue;
        }

        bool color_ok = true;
        if(config.use_color_filter && has_colors) {
            const auto& color = cloud->colors_[i];
            const double dr = color.x() - config.target_r;
            const double dg = color.y() - config.target_g;
            const double db = color.z() - config.target_b;
            const double dist = std::sqrt(dr * dr + dg * dg + db * db);
            color_ok = dist <= config.color_threshold;
        }

        if(!color_ok) {
            continue;
        }

        extracted_cloud->points_.push_back(point);
        if(has_colors) {
            extracted_cloud->colors_.push_back(cloud->colors_[i]);
        }
        if(has_normals) {
            extracted_cloud->normals_.push_back(cloud->normals_[i]);
        }
    }

    if(extracted_cloud->points_.empty()) {
        return nullptr;
    }

    // Skip DBSCAN if user only wants geometric/color masking.
    if(!config.use_clustering) {
        return extracted_cloud;
    }

    const std::vector<int> labels = extracted_cloud->ClusterDBSCAN(config.eps, config.min_points, false);
    std::unordered_map<int, size_t> label_counts;
    int best_label = -1;
    size_t best_count = 0;

    for(const int label : labels) {
        if(label < 0) {
            continue;
        }
        const size_t count = ++label_counts[label];
        if(count > best_count) {
            best_count = count;
            best_label = label;
        }
    }

    if(best_label < 0) {
        return nullptr;
    }

    auto result_cloud = std::make_shared<open3d::geometry::PointCloud>();
    for(size_t i = 0; i < extracted_cloud->points_.size(); ++i) {
        if(labels[i] != best_label) {
            continue;
        }
        result_cloud->points_.push_back(extracted_cloud->points_[i]);
        if(has_colors) {
            result_cloud->colors_.push_back(extracted_cloud->colors_[i]);
        }
        if(has_normals) {
            result_cloud->normals_.push_back(extracted_cloud->normals_[i]);
        }
    }

    return result_cloud;
}

BatchStats ProcessFileRange(
    const std::vector<fs::path>& files,
    size_t start,
    size_t end,
    const Config& config) {
    BatchStats stats;

    // File-level parallel worker: each thread handles a disjoint [start, end) range.
    for(size_t i = start; i < end; ++i) {
        const fs::path& file = files[i];
        ++stats.total;

        auto cloud = std::make_shared<open3d::geometry::PointCloud>();
        if(!open3d::io::ReadPointCloud(file.string(), *cloud)) {
            std::cerr << "[Fail] read point cloud: " << file.string() << std::endl;
            ++stats.failed;
            continue;
        }

        auto result_cloud = ProcessPointCloud(cloud, config);
        if(!result_cloud || result_cloud->points_.empty()) {
            std::cerr << "[Fail] no valid object points: " << file.filename().string() << std::endl;
            ++stats.failed;
            continue;
        }

        const fs::path output_path = fs::path(config.output_dir) / file.filename();
        if(!open3d::io::WritePointCloud(output_path.string(), *result_cloud)) {
            std::cerr << "[Fail] write point cloud: " << output_path.string() << std::endl;
            ++stats.failed;
            continue;
        }

        std::cout << "[OK] " << file.filename().string()
                  << " points: " << cloud->points_.size()
                  << " -> " << result_cloud->points_.size()
                  << " output: " << output_path.string() << std::endl;
        ++stats.success;
    }

    return stats;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Parse config first, then perform directory validation.
    Config config = ParseArguments(argc, argv);

    if(!fs::exists(config.input_dir) || !fs::is_directory(config.input_dir)) {
        std::cerr << "Input directory does not exist: " << config.input_dir << std::endl;
        return 1;
    }

    try {
        fs::create_directories(config.output_dir);
    } catch(const fs::filesystem_error& e) {
        std::cerr << "Failed to create output directory: " << e.what() << std::endl;
        return 1;
    }

    // Collect and sort all matching files for deterministic processing order.
    std::vector<fs::path> files;
    for(const auto& entry : fs::directory_iterator(config.input_dir)) {
        if(!entry.is_regular_file()) {
            continue;
        }
        if(!HasExtensionMatch(entry.path(), config.file_extension)) {
            continue;
        }
        files.push_back(entry.path());
    }

    std::sort(files.begin(), files.end());

    if(files.empty()) {
        std::cout << "No matching files found in: " << config.input_dir << std::endl;
        return 0;
    }

    std::cout << "========== Batch Object Extraction ==========" << std::endl;
    std::cout << "Input directory: " << config.input_dir << std::endl;
    std::cout << "Output directory: " << config.output_dir << std::endl;
    std::cout << "File extension: " << config.file_extension << std::endl;
    std::cout << "Depth threshold: " << config.depth_threshold << " m" << std::endl;
    std::cout << "Color filter: " << (config.use_color_filter ? "enabled" : "disabled") << std::endl;
    std::cout << "Largest-cluster mode: " << (config.use_clustering ? "enabled" : "disabled") << std::endl;
    std::cout << "Threads: " << config.threads << std::endl;

    // Cap worker count by number of files to avoid idle thread creation.
    const unsigned int num_threads = std::max(1u, std::min(config.threads, static_cast<unsigned int>(files.size())));
    const size_t chunk_size = (files.size() + num_threads - 1) / num_threads;

    std::vector<std::thread> workers;
    std::vector<BatchStats> per_thread(num_threads);

    for(unsigned int t = 0; t < num_threads; ++t) {
        const size_t start = t * chunk_size;
        const size_t end = std::min(start + chunk_size, files.size());
        if(start >= end) {
            continue;
        }

        workers.emplace_back([&, t, start, end]() {
            per_thread[t] = ProcessFileRange(files, start, end, config);
        });
    }

    for(auto& worker : workers) {
        worker.join();
    }

    BatchStats total;
    for(const auto& s : per_thread) {
        total.total += s.total;
        total.success += s.success;
        total.failed += s.failed;
    }

    std::cout << "========== Batch Processing Completed ==========" << std::endl;
    std::cout << "Total files: " << total.total << std::endl;
    std::cout << "Succeeded: " << total.success << std::endl;
    std::cout << "Failed: " << total.failed << std::endl;

    return 0;
}
