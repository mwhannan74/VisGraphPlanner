# VisGraphPlanner

A C++ visibility-graph planner for static 2-D polygonal environments.

VisGraphPlanner constructs and uses a visibility graph for path planning around polygonal obstacles. The project is organized as a compiled CMake library with an optional visualization target and a demonstration executable.

## Requirements

- A C++ compiler compatible with the project configuration
- CMake
- Project dependency paths configured in `cmake/local_paths.cmake`

## Build

From the repository root:

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

Dependency paths are configured in:

```text
cmake/local_paths.cmake
```

## Targets

The project provides the following CMake targets:

- `VisGraphPlanner::visgraph` — core visibility-graph planner
- `VisGraphPlanner::visgraph_visualization` — optional visualization support

## Run the Demo

From the build directory on Windows:

```bash
Release\main_demo.exe
```

For single-configuration generators or other platforms, the executable location may differ depending on the selected CMake generator and build configuration.

## Project Scope

VisGraphPlanner is intended for path planning in static, two-dimensional environments represented by polygonal obstacles.

## License

VisGraphPlanner is licensed under the Apache License 2.0.

See [`LICENSE`](LICENSE) for the full license text.

Copyright 2025-2026 Michael Hannan.
