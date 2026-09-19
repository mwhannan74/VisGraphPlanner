# VisGraphPlanner

A C++ visibility-graph planner for static 2-D polygonal environments.

VisGraphPlanner constructs and uses a visibility graph for path planning around polygonal obstacles. The core planner is a header-only CMake target. Optional visualization support and the demonstration executable use [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV).

## How the Planner Works

VisGraphPlanner finds point-to-point routes through a known, static 2-D environment whose obstacles are represented as polygons. It is a geometric path planner rather than a motion controller: its output is a sequence of positions to follow, without vehicle dynamics, turning constraints, or automatic safety clearance. Applications planning for a robot with nonzero size should expand the obstacles by the required footprint and safety margin before building the graph.

The planner treats every obstacle corner as a graph vertex. It connects pairs of vertices when the straight segment between them does not cross an obstacle, while rejecting diagonals that pass through the interior of the same polygon. Each edge is weighted by its Euclidean length. For a planning query, the start and goal are added to the graph and connected to the vertices they can see; Dijkstra's algorithm then returns the lowest-distance route available in the graph.

This approach is simple, deterministic, and effective for relatively small static maps, and it naturally produces direct paths that bend around obstacle corners. Its main tradeoff is scalability: the basic all-pairs visibility build is O(N³) in the number of obstacle vertices and can produce a dense graph. The implementation also assumes valid simple polygons, does not support polygon holes or changing obstacles, and uses floating-point geometric tests that may require care with nearly coincident geometry.

## Assumptions and Limitations

- Obstacles must be represented as simple, counter-clockwise polygons.
- Each polygon must contain at least three vertices.
- Polygon holes are not supported.
- Start and goal points must not be strictly inside an obstacle.
- The obstacle visibility graph is constructed using a naive O(N³) algorithm, where N is the number of obstacle vertices.
- `injectQueryPts()` appends query vertices to the graph. The current API does not provide a method for removing or resetting previously injected query points.

## Run the Demo

From the repository root on Windows with a multi-configuration generator such as Visual Studio:

```powershell
.\build\Release\main_demo.exe
```

**Example Terminal Output:**
```
buildBasic() took 1 ms
shortestPath() took 0 ms
Graph has 915 edges
Path found with 6 waypoints
```

Values vary between runs because the demo randomizes the obstacle positions and start and goal points.

![Visibility graph demo showing the planned path through polygonal obstacles](images/visibility_graph_demo.png)

With a single-configuration generator, the executable is normally `build/main_demo` (`build/main_demo.exe` on Windows). The exact location depends on the selected CMake generator and build configuration.

## Platform Status

This project has been built and tested only on Windows. The code and CMake configuration are intended to be portable to Linux, but Linux has not yet been tested.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- [Eigen](https://eigen.tuxfamily.org/) for the core planner
- [MatPlotOpenCV](https://github.com/mwhannan74/MatPlotOpenCV) for visualization and the demo
- OpenCV 4.5 or newer with the `core`, `highgui`, and `imgproc` components, as required by MatPlotOpenCV
- Doxygen if MatPlotOpenCV documentation is enabled; pass `-DMATPLOTOPENCV_BUILD_DOCS=OFF` if it is not needed

The visualization header includes `figure.h`. That file is provided by MatPlotOpenCV; it is not part of this repository. The core `visibility_graph.hpp` header does not depend on MatPlotOpenCV or OpenCV.

## Configure Dependencies

The default local paths are defined in [`cmake/local_paths.cmake`](cmake/local_paths.cmake):

- `VISGRAPH_EIGEN3_INCLUDE_DIR` points to the Eigen include directory.
- `VISGRAPH_MPOCV_SOURCE_DIR` points to the MatPlotOpenCV source directory containing its `CMakeLists.txt`.

The default MatPlotOpenCV path expects both repositories to have the same parent directory:

```text
parent-directory/
|-- MatPlotOpenCV/
`-- VisGraphPlanner/
```

Edit `cmake/local_paths.cmake` for your machine or override its cached values when configuring:

```bash
cmake -S . -B build -DVISGRAPH_EIGEN3_INCLUDE_DIR=/path/to/eigen -DVISGRAPH_MPOCV_SOURCE_DIR=/path/to/MatPlotOpenCV -DMATPLOTOPENCV_OPENCV_DIR=/path/to/opencv/cmake
```

`MATPLOTOPENCV_OPENCV_DIR` must identify the directory containing `OpenCVConfig.cmake`.

## Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build --config Release
```

To use only the core planner without the visualization dependencies or demo:

```bash
cmake -S . -B build -DVISGRAPH_ENABLE_VISUALIZATION=OFF -DVISGRAPH_BUILD_DEMO=OFF
cmake --build build --config Release
```

MatPlotOpenCV enables its own demo and documentation targets by default. Add `-DMATPLOTOPENCV_BUILD_DEMO=OFF -DMATPLOTOPENCV_BUILD_DOCS=OFF` to the configure command if you do not want to build them.

## Targets

The project provides the following CMake targets:

- `VisGraphPlanner::visgraph` — header-only core visibility-graph planner
- `VisGraphPlanner::visgraph_visualization` — optional visualization support through MatPlotOpenCV
- `main_demo` — demonstration executable, built when `VISGRAPH_BUILD_DEMO=ON`

## Basic Usage

```cpp
#include "visibility_graph.hpp"

#include <iostream>
#include <vector>

int main()
{
    using namespace vg;

    std::vector<Polygon> obstacles{
        {
            Point2(0.0, 0.0),
            Point2(5.0, 0.0),
            Point2(5.0, 5.0),
            Point2(0.0, 5.0)
        }
    };

    VisibilityGraph graph(obstacles);
    graph.buildBasic();

    const Point2 start(-2.0, 2.0);
    const Point2 goal(7.0, 2.0);

    const auto [startId, goalId] = graph.injectQueryPts(start, goal);
    const auto path = graph.shortestPath(startId, goalId);

    std::cout << "Path contains " << path.size() << " points\n";
}
```

## License

VisGraphPlanner is licensed under the Apache License 2.0.

See [`LICENSE`](LICENSE) for the full license text.

Copyright 2025-2026 Michael Hannan.
