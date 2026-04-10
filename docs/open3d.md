# Open3D Integration

This document describes where and how Open3D is used in the toolkit.

For build commands, see [README](../README.md).

## Official Resources

- Open3D repository: [Open3D GitHub](https://github.com/isl-org/Open3D)
- Open3D compilation guide: [Open3D Compilation](https://www.open3d.org/docs/latest/compilation.html)



## Registration and Reconstruction Algorithms

The registration pipeline uses these Open3D operations:

1. `VoxelDownSample` for robust coarse representation.
2. `EstimateNormals` on downsampled clouds.
3. `ComputeFPFHFeature` for feature descriptors.
4. Global registration via one of:
   - `RegistrationRANSACBasedOnFeatureMatching`
   - `RegistrationRANSACBasedOnCorrespondence`
5. Merge and cleanup:
   - `RemoveDuplicatedPoints`
   - `RemoveStatisticalOutliers`
   - Additional voxel downsampling for point-budget control.

## Denoising and Object Filtering

- `filter_object`
  - Crops by front depth slab (`min_z` to `min_z + threshold`)
  - Optional RGB-distance filter
  - Optional DBSCAN with largest-cluster retention
- `filter_isolated_clouds`
  - DBSCAN over each cloud
  - Keeps points only from clusters with `size >= min_cluster_size`

## Visualization

Two visualization entry points are currently available:

- `main --visualize=1`: shows merged final cloud.
- `cloud_point_visualization <path>`: inspect a single file.

## ARM64 and Embedded Platform Notes

On ARM64 devices (for example Raspberry Pi or other embedded Linux boards):

- Prefer building Open3D from source.
- Start from a minimal feature set to reduce build time and memory pressure.
- Use release build (`-DCMAKE_BUILD_TYPE=Release`) for runtime performance.
- If Open3D package discovery fails, verify `Open3D_DIR` in CMake cache.

Example CMake configure command:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpen3D_DIR=/path/to/open3d/install/lib/cmake/Open3D \
  -DORBBECSDK_ROOT=/path/to/OrbbecSDK
```
## Open3D Usage by Module

| Module | Open3D features used |
| --- | --- |
| `sample_data` | No direct Open3D API call in current source (capture and PLY writing are done via Orbbec SDK + custom writer) |
| `filter_object` | Point cloud IO, geometric filtering, DBSCAN clustering |
| `filter_isolated_clouds` | Point cloud IO, DBSCAN clustering, index-based point selection |
| `ransac_registration` + `main` | Voxel downsampling, normal estimation, FPFH features, RANSAC registration, outlier removal, visualization |
| `cloud_point_visualization` | Point cloud IO and interactive rendering |

## Open3D APIs used in this repository (`./src/*.cpp`)

The following table maps concrete Open3D APIs to the source modules where they are called.

| API | Used in source files | What it does in this project |
| --- | --- | --- |
| `open3d::io::CreatePointCloudFromFile` | `main.cpp`, `ransac_registration.cpp`, `filter_isolated_clouds.cpp` | Load point clouds for filtering/registration |
| `open3d::io::ReadPointCloud` | `cloud_point_visualization.cpp`, `filter_object.cpp` | Read a cloud into an existing object |
| `open3d::io::WritePointCloud` | `main.cpp`, `filter_isolated_clouds.cpp`, `filter_object.cpp` | Persist filtered/merged outputs |
| `VoxelDownSample` | `ransac_registration.cpp`, `main.cpp` | Reduce density for registration and memory guard |
| `EstimateNormals` | `ransac_registration.cpp` | Prepare normals for feature computation |
| `ComputeFPFHFeature` | `ransac_registration.cpp` | Build descriptors for global registration |
| `RegistrationRANSACBasedOnFeatureMatching` | `ransac_registration.cpp` | Global registration from feature matches |
| `RegistrationRANSACBasedOnCorrespondence` | `ransac_registration.cpp` | Global registration from explicit correspondences |
| `ClusterDBSCAN` | `filter_object.cpp`, `filter_isolated_clouds.cpp` | Keep dominant object / remove isolated clusters |
| `SelectByIndex` | `filter_isolated_clouds.cpp` | Rebuild cloud from selected point indices |
| `RemoveDuplicatedPoints` | `main.cpp` | Remove duplicate points after merge |
| `RemoveStatisticalOutliers` | `main.cpp` | Post-merge denoising |
| `DrawGeometries` | `main.cpp`, `cloud_point_visualization.cpp`, `ransac_registration.cpp` | Interactive visualization for debug/final results |

### Minimal example: IO + downsample + save

```cpp
auto cloud = open3d::io::CreatePointCloudFromFile("input.ply");
if(cloud && !cloud->IsEmpty()) {
  auto down = cloud->VoxelDownSample(0.005);
  if(down && !down->IsEmpty()) {
    open3d::io::WritePointCloud("output_downsampled.ply", *down);
  }
}
```

### Minimal example: FPFH + RANSAC feature matching

```cpp
auto src = open3d::io::CreatePointCloudFromFile("src.ply");
auto dst = open3d::io::CreatePointCloudFromFile("dst.ply");

double voxel = 0.01;
auto src_down = src->VoxelDownSample(voxel);
auto dst_down = dst->VoxelDownSample(voxel);

src_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(2.0 * voxel, 30));
dst_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(2.0 * voxel, 30));

auto src_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
  *src_down, open3d::geometry::KDTreeSearchParamHybrid(5.0 * voxel, 100));
auto dst_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
  *dst_down, open3d::geometry::KDTreeSearchParamHybrid(5.0 * voxel, 100));

auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
  *src_down,
  *dst_down,
  *src_fpfh,
  *dst_fpfh,
  false,
  voxel * 1.5,
  open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
  3,
  {},
  open3d::pipelines::registration::RANSACConvergenceCriteria(100000, 0.999));
```

### Minimal example: DBSCAN keep largest cluster

```cpp
auto cloud = open3d::io::CreatePointCloudFromFile("filtered_input.ply");
auto labels = cloud->ClusterDBSCAN(0.02, 20, false);

std::vector<size_t> keep;
std::unordered_map<int, int> count;
for(int l : labels) if(l >= 0) ++count[l];

int best = -1, best_n = 0;
for(auto &kv : count) {
  if(kv.second > best_n) {
    best = kv.first;
    best_n = kv.second;
  }
}

for(size_t i = 0; i < labels.size(); ++i) {
  if(labels[i] == best) {
    keep.push_back(i);
  }
}

auto largest = cloud->SelectByIndex(keep);
open3d::io::WritePointCloud("largest_cluster.ply", *largest);
```

## Common Open3D APIs for extension

The following APIs are not all used in current code, but are practical when extending this project.

| API | Typical use |
| --- | --- |
| `RemoveRadiusOutliers` | Radius-based denoising alternative to statistical outlier removal |
| `UniformDownSample` | Keep every k-th point for fast previews |
| `Transform` | Apply known 4x4 pose transforms to clouds |
| `PaintUniformColor` | Assign temporary debug colors per cloud |
| `GetAxisAlignedBoundingBox` / `Crop` | ROI crop before heavy processing |
| `KDTreeFlann` | Nearest-neighbor search for custom matching |

### Minimal example: transform + color + visualize

```cpp
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
T(0, 3) = 0.1;  // translate 10 cm on x

auto cloud = open3d::io::CreatePointCloudFromFile("registered_cloud.ply");
cloud->Transform(T);
cloud->PaintUniformColor(Eigen::Vector3d(0.1, 0.7, 0.2));
open3d::visualization::DrawGeometries({cloud}, "Debug View");
```
## Related Docs

- Pipeline details: [reconstruction.md](reconstruction.md)
- Developer extension guide: [developer.md](developer.md)
