#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <vector>

#include <open3d/Open3D.h>

namespace {

// Batch denoising tool.
//
// This executable processes all clouds in a directory and removes isolated
// small clusters using DBSCAN labels + minimum cluster-size filtering.

// Filesystem helpers.
bool IsDirectory(const std::string &path) {
    struct stat st;
    if(stat(path.c_str(), &st) != 0) {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

bool EnsureDirectory(const std::string &path) {
    if(IsDirectory(path)) {
        return true;
    }
    if(mkdir(path.c_str(), 0755) == 0) {
        return true;
    }
    if(errno == EEXIST) {
        return IsDirectory(path);
    }
    return false;
}

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool HasSupportedPointCloudExtension(const std::string &filename) {
    auto pos = filename.find_last_of('.');
    if(pos == std::string::npos) {
        return false;
    }
    std::string ext = ToLower(filename.substr(pos));
    return ext == ".ply" || ext == ".pcd" || ext == ".xyz" || ext == ".xyzn" || ext == ".xyzrgb";
}

std::string JoinPath(const std::string &a, const std::string &b) {
    if(a.empty()) {
        return b;
    }
    if(a.back() == '/') {
        return a + b;
    }
    return a + "/" + b;
}

void PrintUsage(const char *program) {
    std::cout << "Usage:\n"
              << "  " << program << " <input_dir> [--eps=0.02] [--min_points=20] "
              << "[--min_cluster_size=200] [--output_dir=<path>] [--inplace]\n\n"
              << "Description:\n"
              << "  Filter small isolated point-cloud clusters for all point-cloud files in a directory.\n"
              << "  Supported extensions: .ply .pcd .xyz .xyzn .xyzrgb\n"
              << "  Default output directory: /home/pi/repo/orbbec_reconstruction/data/filtered_data\n"
              << "  Use --inplace to overwrite original files.\n\n"
              << "Options:\n"
              << "  --eps=<float>               DBSCAN epsilon radius in meters (default: 0.02)\n"
              << "  --min_points=<int>          DBSCAN minimum points per neighborhood (default: 20)\n"
              << "  --min_cluster_size=<int>    Keep clusters with at least this many points (default: 200)\n"
              << "  --output_dir=<path>         Output directory when not using --inplace\n"
              << "  --inplace                   Overwrite input files\n"
              << "  -h, --help                  Show this help message\n";
}

// Worker function for a file-index range. Returns number of successful outputs.
int ProcessFileRange(const std::vector<std::string>& file_names,
                     size_t start, size_t end,
                     const std::string& input_dir,
                     const std::string& output_dir,
                     bool inplace,
                     double eps,
                     int min_points,
                     int min_cluster_size) {
    int success = 0;
    for (size_t idx = start; idx < end; ++idx) {
        const std::string& name = file_names[idx];
        std::string input_path = JoinPath(input_dir, name);
        std::string output_path = inplace ? input_path : JoinPath(output_dir, name);

        try {
            auto cloud = open3d::io::CreatePointCloudFromFile(input_path);
            if(!cloud || cloud->IsEmpty()) {
                std::cout << "[Skip] " << name << " (failed to load or empty)" << std::endl;
                continue;
            }

            // Run DBSCAN clustering over the current cloud.
            auto labels = cloud->ClusterDBSCAN(eps, min_points, false);
            if(labels.empty()) {
                std::cout << "[Skip] " << name << " (dbscan returned no labels)" << std::endl;
                continue;
            }

            int max_label = -1;
            for(int label : labels) {
                if(label > max_label) {
                    max_label = label;
                }
            }

            std::vector<int> cluster_sizes;
            if(max_label >= 0) {
                cluster_sizes.assign(static_cast<size_t>(max_label + 1), 0);
                for(int label : labels) {
                    if(label >= 0) {
                        ++cluster_sizes[static_cast<size_t>(label)];
                    }
                }
            }

            // Keep only points that belong to large-enough clusters.
            // Labels with -1 are DBSCAN noise and are discarded.
            std::vector<size_t> keep_indices;
            for(size_t i = 0; i < labels.size(); ++i) {
                int label = labels[i];
                if(label >= 0 && cluster_sizes[static_cast<size_t>(label)] >= min_cluster_size) {
                    keep_indices.push_back(i);
                }
            }

            auto filtered = cloud->SelectByIndex(keep_indices);
            if(!filtered) {
                std::cout << "[Fail] " << name << " (failed to build filtered cloud)" << std::endl;
                continue;
            }

            if(!open3d::io::WritePointCloud(output_path, *filtered)) {
                std::cout << "[Fail] " << name << " (failed to write file)" << std::endl;
                continue;
            }

            ++success;
            std::cout << "[OK] " << name
                      << " points: " << cloud->points_.size()
                      << " -> " << filtered->points_.size()
                      << " output: " << output_path << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "[Exception] " << name << ": " << e.what() << std::endl;
        }
        catch (...) {
            std::cout << "[Exception] " << name << ": unknown error" << std::endl;
        }
    }
    return success;
}

}  // namespace

int main(int argc, char *argv[]) {
    if(argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    // Parse command-line arguments.
    std::string input_dir = argv[1];
    double eps = 0.02;
    int min_points = 20;
    int min_cluster_size = 200;
    std::string output_dir = "/home/pi/repo/orbbec_reconstruction/data/filtered_data";
    std::string output_dir_arg;
    bool inplace = false;

    for(int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg.rfind("--eps=", 0) == 0) {
            eps = std::stod(arg.substr(6));
        }
        else if(arg.rfind("--min_points=", 0) == 0) {
            min_points = std::stoi(arg.substr(13));
        }
        else if(arg.rfind("--min_cluster_size=", 0) == 0) {
            min_cluster_size = std::stoi(arg.substr(19));
        }
        else if(arg.rfind("--output_dir=", 0) == 0) {
            output_dir_arg = arg.substr(13);
        }
        else if(arg == "--inplace") {
            inplace = true;
        }
        else if(arg == "-h" || arg == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
        else {
            std::cerr << "Unknown option: " << arg << std::endl;
            PrintUsage(argv[0]);
            return 1;
        }
    }

    // Validate arguments.
    if(eps <= 0.0 || min_points <= 0 || min_cluster_size <= 0) {
        std::cerr << "Invalid parameters: eps, min_points, min_cluster_size must be > 0." << std::endl;
        return 1;
    }

    // Validate input directory.
    if(!IsDirectory(input_dir)) {
        std::cerr << "Input is not a directory: " << input_dir << std::endl;
        return 1;
    }

    // Resolve output directory.
    if(!inplace) {
        if(!output_dir_arg.empty()) {
            output_dir = output_dir_arg;
        }

        // Ensure output directory exists.
        if(!EnsureDirectory(output_dir)) {
            std::cerr << "Failed to create output directory: " << output_dir
                      << " (" << std::strerror(errno) << ")" << std::endl;
            return 1;
        }
    }

    // Collect point-cloud files.
    DIR *dir = opendir(input_dir.c_str());
    if(!dir) {
        std::cerr << "Failed to open directory: " << input_dir
                  << " (" << std::strerror(errno) << ")" << std::endl;
        return 1;
    }

    std::vector<std::string> files;
    struct dirent *entry = nullptr;
    while((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if(name == "." || name == "..") continue;
        if(!HasSupportedPointCloudExtension(name)) continue;

        std::string full_path = JoinPath(input_dir, name);
        struct stat st;
        if(stat(full_path.c_str(), &st) == 0 && S_ISREG(st.st_mode)) {
            files.push_back(name);
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end());

    if(files.empty()) {
        std::cout << "No supported point-cloud files found in: " << input_dir << std::endl;
        return 0;
    }

    std::cout << "Found " << files.size() << " point-cloud files." << std::endl;
    std::cout << "Filtering params: eps=" << eps
              << ", min_points=" << min_points
              << ", min_cluster_size=" << min_cluster_size
              << ", inplace=" << (inplace ? "true" : "false") << std::endl;

    // ----- Multi-threaded processing -----
    // Work is split into contiguous file chunks.
    unsigned int num_threads = std::thread::hardware_concurrency() / 2;
    if (num_threads == 0) num_threads = 2;   // Safe fallback.

    size_t total_files = files.size();
    size_t chunk_size = (total_files + num_threads - 1) / num_threads;  // Round up.

    std::vector<std::thread> threads;
    std::vector<int> thread_success(num_threads, 0);

    for (unsigned int t = 0; t < num_threads; ++t) {
        size_t start = t * chunk_size;
        size_t end = std::min(start + chunk_size, total_files);
        if (start >= end) continue;   // No files assigned to this thread.

        // Launch worker thread for this chunk.
        threads.emplace_back([&, t, start, end]() {
            thread_success[t] = ProcessFileRange(files, start, end,
                                                  input_dir, output_dir,
                                                  inplace, eps, min_points, min_cluster_size);
        });
    }

    // Wait for all worker threads.
    for (auto& th : threads) {
        th.join();
    }

    int succeeded = 0;
    for (int s : thread_success) succeeded += s;

    std::cout << "Done. success=" << succeeded << "/" << total_files << std::endl;
    return 0;
}