# Developer Guide

This guide targets developers who want to modify or extend the toolkit.

For environment setup and baseline build commands, see [README](../README.md).

## Code Structure

```text
include/
  ransac_registration.hpp
src/
  sample_data.cpp
  filter_object.cpp
  filter_isolated_clouds.cpp
  ransac_registration.cpp
  main.cpp
  cloud_point_visualization.cpp
CMakeLists.txt
```

## Add a New Executable

1. Create a new source file in `src/`, for example `src/my_new_stage.cpp`.
2. Register target in `CMakeLists.txt`.
3. Link against required libraries (`Open3D::Open3D`, `Eigen3::Eigen`, optional Orbbec SDK).

Example CMake block:

```cmake
add_executable(my_new_stage
    src/my_new_stage.cpp
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.1)
    target_compile_options(my_new_stage PRIVATE -Wno-psabi)
endif()

target_include_directories(my_new_stage
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ORBBECSDK_INCLUDE_DIR}
)

target_link_libraries(my_new_stage
    PRIVATE
        Open3D::Open3D
        Eigen3::Eigen
        ${ORBBECSDK_LIBRARY}
)
```

If your executable does not access camera APIs, linking Orbbec SDK is optional from a functional perspective.

## Add a New Filter Stage

Recommended pattern:

1. Parse arguments clearly and print usage.
2. Enumerate files deterministically (sorted order).
3. Keep input/output contract stable (preserve filename when possible).
4. Use per-file processing with optional multithreading.
5. Log compact per-file summary and final stats.

Where to integrate:

- Standalone stage before `main` for preprocessing.
- Or add helper functions in `src/main.cpp` for post-merge cleanup.

## Add a New Registration Method

Current registration abstraction is defined in `include/ransac_registration.hpp`.

Extension path:

1. Add new option value to `RansacRegistrationOptions::method` semantics.
2. Implement branch in `RunRansacRegistrationFromFiles` in `src/ransac_registration.cpp`.
3. Extend argument parser and help text in `src/main.cpp`.
4. Keep transformation validity checks and error propagation consistent.

Implementation guidelines:

- Validate method-specific parameters early.
- Reuse existing preprocessing unless algorithm requires a different feature pipeline.
- Keep output compatible: a finite 4x4 transform mapping source to target frame.

## Add or Improve Visualization

Existing options:

- Final merged cloud viewer in `main` (`--visualize=1`).
- Single-file viewer in `cloud_point_visualization`.

Common enhancements:

- Overlay source/target clouds with distinct colors.
- Add coordinate frame visualization.
- Add optional camera trajectory or frame index annotations.

## CMake and Linking Tips

- Required packages in current build:
  - Open3D
  - Eigen3
  - Orbbec SDK (currently configured as required globally)
- If SDK is installed in non-standard location:
  - configure with `-DORBBECSDK_ROOT=/path/to/sdk`
- On ARM64, ensure Open3D is compiled for the same toolchain and ABI as this project.

Example configure:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DORBBECSDK_ROOT=/opt/OrbbecSDK \
  -DOpen3D_DIR=/path/to/open3d/install/lib/cmake/Open3D
cmake --build build -j8
```

## Conventions Observed in Current Code

- C++14, straightforward procedural pipeline in executables.
- Clear CLI help strings for each tool.
- Defensive checks before expensive processing.
- Multi-threading used in batch filters at file granularity.
- Output filenames are intentionally stable to support shell scripting.

## Suggested Development Workflow

1. Implement feature in isolated executable first.
2. Validate on small cloud subset.
3. Integrate into end-to-end pipeline.
4. Document CLI contract in docs and README as needed.

## Cross References

- System overview: [overview.md](overview.md)
- SDK-specific details: [orbbecsdk.md](orbbecsdk.md)
- Open3D details: [open3d.md](open3d.md)
- Pipeline execution contract: [reconstruction.md](reconstruction.md)
