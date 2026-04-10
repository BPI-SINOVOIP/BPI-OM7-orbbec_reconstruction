# Orbbec Reconstruction Toolkit: Developer Overview

This document describes the architecture and execution model of the Orbbec Reconstruction Toolkit for developers.

For quick-start build and basic usage, see [README](../README.md).

## Scope

The project provides a modular point-cloud pipeline implemented as standalone C++ executables:

1. Capture from Orbbec camera (`sample_data`)
2. Object extraction (`filter_object`)
3. Isolated-cluster removal (`filter_isolated_clouds`)
4. Multi-view registration and merge (`main`)
5. Visualization (`cloud_point_visualization`)

## End-to-End Workflow

```text
sample_data
  input: live camera streams (depth + color)
  output: raw PLY files (data_0.ply, data_1.ply, ...)
    │
    ▼
filter_object (optional)
  input: raw point clouds directory
  output: object-focused point clouds
    │
    ▼
filter_isolated_clouds (optional)
  input: filtered clouds directory
  output: denoised clouds
    │
    ▼
main
  input: indexed point cloud sequence
  output: registered_cloud.ply
    │
    ▼
cloud_point_visualization
  input: any cloud file (PLY/PCD/etc. supported by Open3D reader)
  output: interactive viewer window
```

## Executable Chain Diagram

```text
sample_data -> filter_object -> filter_isolated_clouds -> main -> cloud_point_visualization
```

## Runtime Responsibilities by Component

| Executable | Primary responsibility | Typical input | Typical output |
| --- | --- | --- | --- |
| `sample_data` | Acquire synchronized depth/color frames, generate RGB point cloud, write PLY | Orbbec device stream | `data/.../raw_data/data_*.ply` |
| `filter_object` | Keep near-front object region, optional color mask and largest-cluster selection | Directory of PLY files | Directory of object-focused clouds |
| `filter_isolated_clouds` | Run DBSCAN and remove small isolated clusters | Directory of cloud files | Denoised clouds (in-place or new directory) |
| `main` | Pairwise registration + chained transform merge + cleanup + save final cloud | Indexed cloud sequence | `result_dir/registered_cloud.ply` |
| `cloud_point_visualization` | Inspect single cloud interactively | One cloud path | Viewer window |

## Key Integration Points

- Orbbec SDK usage details: [orbbecsdk.md](orbbecsdk.md)
- Open3D algorithm and API usage: [open3d.md](open3d.md)
- Full pipeline execution details and file conventions: [reconstruction.md](reconstruction.md)
- Extension and customization guide: [developer.md](developer.md)

## Notes for Developers

- The registration executable (`main`) expects indexed filenames: `<prefix><index><ext>`.
- Capture output prefix is `data_` by default, while registration default prefix is `data_`.
- You can avoid renaming by passing `--file_prefix=data_` to `main`.
