# 3D Reconstruction Pipeline Details

This document explains the reconstruction flow as implemented in the current codebase.

For quick-start commands, see [README](../README.md).

## Pipeline Summary

```text
Capture -> Object Filter -> Isolated-Cluster Cleanup -> Pairwise Registration -> Global Merge -> Visualization
```

## Data Flow Diagram

```text
live camera
  -> sample_data
  -> raw_data/data_*.ply
  -> filter_object
  -> filtered_data/*.ply
  -> filter_isolated_clouds (optional)
  -> filtered_data_clean/*.ply or inplace overwrite
  -> main
  -> result/registered_cloud.ply
  -> cloud_point_visualization
```

## Stage-by-Stage Execution

## 1) Capture Point Clouds

Command:

```bash
./sample_data manual ../data/object_data/raw_data
# or
./sample_data timer 1000 ../data/object_data/raw_data
```

Input:

- Live Orbbec depth + color streams.

Output:

- Indexed ASCII PLY sequence: `data_0.ply`, `data_1.ply`, ...

Notes:

- Manual mode saves on `R/r` key.
- Timer mode saves at fixed interval.

## 2) Extract Object Region

Command:

```bash
./filter_object \
  --input-dir ../data/object_data/raw_data \
  --output-dir ../data/object_data/filtered_data \
  --depth-threshold 0.6 \
  --cluster 0.03 20 \
  --threads 4
```

Input:

- Raw per-view point cloud files.

Output:

- Filtered per-view clouds in output directory, preserving original filenames.

Processing logic:

1. Compute `min_z` of each cloud.
2. Keep points in `[min_z, min_z + depth_threshold]`.
3. Optionally keep only points near target RGB.
4. Optionally run DBSCAN and retain largest cluster.

Method limitations (`src/filter_object.cpp`):

This method is designed for practical object capture in relatively clean scenes.

- Assumes the object is the closest surface to the camera
- Uses a fixed depth threshold per frame
- Optional color filtering depends on lighting conditions
- Largest-cluster selection may remove valid object parts
- Frames are processed independently without tracking

For complex scenes, manual tuning or more advanced segmentation
methods may be required.

## 3) Remove Small Isolated Clusters (Optional)

Command:

```bash
./filter_isolated_clouds ../data/object_data/filtered_data \
  --eps=0.02 \
  --min_points=20 \
  --min_cluster_size=200 \
  --inplace
```

Input:

- Filtered clouds from stage 2.

Output:

- Denoised clouds, either overwritten (`--inplace`) or written to `--output_dir`.

## 4) Register and Merge Multi-View Clouds

Command:

```bash
./main \
  --input_data_dir=../data/object_data/filtered_data \
  --result_dir=../data/object_data/result \
  --file_prefix=data_ \
  --file_ext=.ply \
  --start_index=0 \
  --end_index=-1 \
  --method=feature_matching \
  --voxel_size=0.005 \
  --visualize=1
```

Input contract:

- File pattern: `<input_data_dir>/<file_prefix><index><file_ext>`
- Example: `../data/object_data/filtered_data/data_0.ply`

Output:

- `registered_cloud.ply` written to `result_dir`.

Registration logic:

1. Load adjacent cloud pair: `(i, i-1)`.
2. Preprocess each cloud:
   - voxel downsample
   - normal estimation
   - FPFH feature computation
3. Compute global transform via selected RANSAC method.
4. Chain transforms into base frame.
5. Merge transformed source cloud into accumulator.
6. Apply memory guard downsampling when point count grows too large.
7. Final cleanup:
   - remove non-finite points
   - remove duplicated points
   - statistical outlier removal
   - output point budget downsampling

## 5) Visualize

Command:

```bash
./cloud_point_visualization ../data/object_data/result/registered_cloud.ply
```

## File Naming Conventions

| Stage | Default naming | Notes |
| --- | --- | --- |
| Capture | `data_<index>.ply` | Produced by `sample_data` |
| Registration input | `<prefix><index><ext>` | `main` default prefix is `data_` |
| Final output | `registered_cloud.ply` | Written by `main` |

Important compatibility note:

- If capture output is `data_*.ply`, pass `--file_prefix=data_` to `main`.

## Typical Directory Layout

```text
data/
├── object_data/
│   ├── raw_data/
│   │   ├── data_0.ply
│   │   └── data_1.ply
│   ├── filtered_data/
│   │   ├── data_0.ply
│   │   └── data_1.ply
│   └── result/
│       └── registered_cloud.ply
```

## Failure Modes to Check First

- Fewer than two valid input clouds for `main`.
- Prefix/index mismatch causing missing files.
- Overly small `voxel_size` increasing runtime significantly.
- Overly strict DBSCAN parameters removing most points.

## Related Docs

- High-level architecture: [overview.md](overview.md)
- Orbbec camera integration: [orbbecsdk.md](orbbecsdk.md)
- Open3D algorithms and build notes: [open3d.md](open3d.md)
- Extension guide: [developer.md](developer.md)
