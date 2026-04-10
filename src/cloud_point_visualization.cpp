#include <iostream>
#include <memory>
#include <open3d/Open3D.h>

namespace {

// Simple viewer utility for inspecting a single point-cloud file.
// This tool is intentionally minimal: load file -> print metadata -> open viewer.
void PrintUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " <point_cloud_path>\n\n"
        << "Arguments:\n"
        << "  <point_cloud_path>    Input point-cloud file path (for example .ply, .pcd)\n"
        << "  -h, --help            Show this help message\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    // Initialize Open3D logging before any IO/visualization calls.
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    if(argc >= 2) {
        const std::string arg1 = argv[1];
        if(arg1 == "-h" || arg1 == "--help") {
            PrintUsage(argv[0]);
            return 0;
        }
    }

    // Check command line arguments.
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }
    
    std::string ply_file_path = argv[1];
    
    // Read input point cloud from disk.
    std::cout << "Reading PLY file: " << ply_file_path << std::endl;
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    
    if (!open3d::io::ReadPointCloud(ply_file_path, *cloud)) {
        std::cerr << "Failed to read PLY file!" << std::endl;
        return 1;
    }
    
    // Print metadata to verify content before rendering.
    std::cout << "Successfully read point cloud:" << std::endl;
    std::cout << "Number of points: " << cloud->points_.size() << std::endl;
    
    if (cloud->HasColors()) {
        std::cout << "Contains color information" << std::endl;
    }
    
    if (cloud->HasNormals()) {
        std::cout << "Contains normal information" << std::endl;
    }
    
    // Open interactive window for manual inspection.
    std::cout << "Visualizing point cloud..." << std::endl;
    open3d::visualization::DrawGeometries({cloud}, "PLY File Visualization", 800, 600);
    
    return 0;
}