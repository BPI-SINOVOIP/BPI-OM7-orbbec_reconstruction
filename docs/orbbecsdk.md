# Orbbec SDK Integration

This document explains how the project uses Orbbec SDK in the current codebase.

For build prerequisites and quick start, see [README](../README.md).

## Official Resources

- Orbbec SDK repository: [OrbbecSDK](https://github.com/orbbec/OrbbecSDK)
- Orbbec SDK releases: [OrbbecSDK_v2 Releases](https://github.com/orbbec/OrbbecSDK_v2/releases)

## Which Executables Use Orbbec SDK

### Direct runtime dependency

- `sample_data`
  - Uses Orbbec device APIs to open pipeline, configure streams, wait for frames, and generate point clouds.

### Build-time linked but not actively used for device access

- `filter_object`
- `filter_isolated_clouds`
- `main`
- `cloud_point_visualization`

In the current `CMakeLists.txt`, all executables are linked with the Orbbec SDK library. Functionally, only `sample_data` requires live SDK interaction.

## SDK Responsibilities in `sample_data`

`sample_data` handles the camera side of the pipeline:

1. Initialize SDK logging and pipeline.
2. Query color stream profile.
3. Select depth profile aligned to color stream.
4. Enable frame sync when supported.
5. Start pipeline and continuously fetch frames.
6. Convert depth+color frames into RGB point cloud via `ob::PointCloudFilter`.
7. Save ASCII PLY files (`data_<index>.ply`).

## Capture Modes

```bash
./sample_data manual [raw_data_dir]
./sample_data timer <interval_ms> [raw_data_dir]
```

- `manual`: save when pressing `R/r`; `ESC` exits.
- `timer`: auto-save every `interval_ms` milliseconds.

## Output Contract for Downstream Stages

- Default output file naming: `data_0.ply`, `data_1.ply`, ...
- Default output directory in source: `/home/pi/repo/orbbec_reconstruction/data/raw_data`
- Coordinates are written in meters using scale factor `0.001` from SDK point units.

Since `main` defaults to prefix `data_`, use one of these options:

1. Rename captured files from `data_*.ply` to `data_*.ply`.
2. Run registration with `--file_prefix=data_`.

Example:

```bash
./main --input_data_dir=../data/object_data/filtered_data \
       --file_prefix=data_ \
       --result_dir=../data/object_data/result
```

## Device and Alignment Behavior

The capture tool attempts alignment in this order:

1. Hardware depth-to-color alignment (`ALIGN_D2C_HW_MODE`)
2. Software depth-to-color alignment (`ALIGN_D2C_SW_MODE`) as fallback

If frame sync is unsupported by the device, the tool logs a warning and continues.

## Troubleshooting

- SDK not found at configure time:
  - Set `-DORBBECSDK_ROOT=<path>` or export `ORBBECSDK_ROOT`.
- No color sensor support:
  - `sample_data` exits with an explicit error.
- Empty/invalid frames:
  - Manual mode retries frame fetch several times before skipping save.

## Common Orbbec SDK APIs (from official examples)

The following APIs are commonly used in the [OrbbecSDK](https://github.com/orbbec/OrbbecSDK) examples, and are also aligned with this project's capture flow.

| API | Purpose | Example locations in OrbbecSDK examples |
| --- | --- | --- |
| `ob::Context` | SDK context, device discovery, global setup | `examples/CommonUsages`, `examples/NetDevice` |
| `ob::Pipeline` | Stream pipeline for sensors and frame retrieval | `examples/QuickStart`, `examples/Transformation` |
| `ob::Config` | Enable stream profiles and set alignment mode | `examples/CommonUsages`, `examples/InfraredViewer` |
| `getStreamProfileList` | Query available stream profiles by sensor type | `examples/InfraredViewer`, `examples/NetDevice` |
| `enableStream` | Enable depth/color/IR stream in config | most streaming samples |
| `setAlignMode` | Configure depth-color alignment behavior | `examples/Transformation` |
| `start` / `stop` | Start or stop streaming pipeline | most streaming samples |
| `waitForFrames` | Poll a synchronized frame set | `examples/QuickStart`, `examples/Transformation` |
| `depthFrame` / `colorFrame` | Access depth or color frame from frameset | `examples/CommonUsages`, `examples/Transformation` |
| `ob::PointCloudFilter` | Convert frameset into point cloud frames | `examples/PointCloud`, `examples/Transformation` |
| `getCameraParam` | Retrieve camera intrinsics/extrinsics for filters | `examples/CommonUsages`, `examples/Transformation` |
| `enableFrameSync` | Improve frame timestamp sync if supported | seen in pipeline-based examples and this project |

### Minimal API example: pipeline + stream setup + frame polling

```cpp
#include "libobsensor/ObSensor.hpp"

ob::Pipeline pipe;
auto config = std::make_shared<ob::Config>();

auto color_profiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
auto color_profile = color_profiles->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
config->enableStream(color_profile);

auto depth_profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
auto depth_profile = depth_profiles->getProfile(OB_PROFILE_DEFAULT);
config->enableStream(depth_profile);

config->setAlignMode(ALIGN_D2C_HW_MODE);
pipe.start(config);

auto frameset = pipe.waitForFrames(100);
if(frameset && frameset->depthFrame() && frameset->colorFrame()) {
  // Use depth/color frames here.
}

pipe.stop();
```

### Minimal API example: generate point cloud from frameset

```cpp
ob::PointCloudFilter pc_filter;
pc_filter.setCameraParam(pipe.getCameraParam());

auto depth_scale = frameset->depthFrame()->getValueScale();
pc_filter.setPositionDataScaled(depth_scale);
pc_filter.setCreatePointFormat(OB_FORMAT_RGB_POINT);

std::shared_ptr<ob::Frame> points = pc_filter.process(frameset);
if(points) {
  // points->data() contains OBColorPoint buffer.
}
```

### Practical tip

- In this project, `sample_data` already implements the above pattern with fallback alignment logic and manual/timer save modes. See [reconstruction.md](reconstruction.md) for full pipeline context.
