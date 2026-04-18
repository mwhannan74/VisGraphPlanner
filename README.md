# VisGraphPlanner
A visibility graph (VG) for static 2‑D polygonal environments.

## Build:
```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

Dependency paths are configured in `cmake/local_paths.cmake`.

Targets:
- `VisGraphPlanner::visgraph` for the core planner
- `VisGraphPlanner::visgraph_visualization` for optional plotting support

## Run:
From the build directory
```bash
Release\main_demo.exe
```
