# gtsamGNSS

Standalone C++20 GNSS/robust-factor library extracted from the legacy robust GNSS fork,
intended for integration with modern GTSAM 4.x builds.

## Scope

- `gtsam/gnssNavigation`: GNSS factors, tools, data helpers, and state wrappers.
- `gtsam/robustModels`: switch/max-mix robust factor variants.

## Build

```bash
cmake --preset macos-debug
cmake --build --preset build-macos-debug -j
ctest --preset test-macos-debug
```

Linux presets:

```bash
cmake --preset linux-release
cmake --build --preset build-linux-release -j
```

Strict/sanitizer presets:

```bash
cmake --preset linux-debug-strict
cmake --build --preset build-linux-debug-strict -j
ctest --preset test-linux-debug-strict

cmake --preset linux-asan
cmake --build --preset build-linux-asan -j
ctest --preset test-linux-asan
```

If CMake cannot find GTSAM automatically, set your config path first:

```bash
export GTSAM_DIR=/path/to/gtsam/lib/cmake/GTSAM
```

## Design Notes

- `PhaseBias` and multimodal component types are centralized in
  `include/gtsam/gnssNavigation/GnssTypes.h`.
- `GNSSMultiModalFactor` no longer depends on `libcluster` types.
- Public headers avoid global `using namespace` leakage.
- Typed GNSS I/O errors are defined in
  `include/gtsam/gnssNavigation/GnssErrors.h`.
