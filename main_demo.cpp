// main_demo.cpp – Simple demo of the VisibilityGraph class with built-in
// visualisation (mpocv::Figure).
//
// Build example (Linux, GCC):
//   g++ -std=c++17 -O2 main_demo.cpp \
//       -I/path/to/eigen \
//       -I/path/to/mpocv/include \
//       -L/path/to/mpocv/lib -lmpocv  -o demo
//
// Adjust include/library paths and linker flags for your platform.
// ---------------------------------------------------------------------------

#include "visibility_graph.hpp"
#include <iostream>

using namespace vg;

int main()
{
    /* --------------------------------------------------------------------
       1. Define polygonal obstacles (counter-clockwise). Here two rectangles.
    -------------------------------------------------------------------- */
    Polygon box1 = { {0,0}, {4,0}, {4,2}, {0,2} };
    Polygon box2 = { {6,1}, {9,1}, {9,4}, {6,4} };
    std::vector<Polygon> obstacles = { box1, box2 };

    /* --------------------------------------------------------------------
       2. Construct and build the visibility graph
    -------------------------------------------------------------------- */
    VisibilityGraph vg(obstacles);
    vg.build();

    /* --------------------------------------------------------------------
       3. Insert start (S) and goal (G) points and solve for shortest path
    -------------------------------------------------------------------- */
    Point2 S(-1, 1);
    Point2 G(10, 3);

    auto [sid, gid] = vg.injectQueryPts(S, G);
    auto path = vg.shortestPath(sid, gid);

    /* --------------------------------------------------------------------
       4. Visualise the graph and the path
    -------------------------------------------------------------------- */
    vg.visualize(S, G, path);

    /* --------------------------------------------------------------------
       5. Console output
    -------------------------------------------------------------------- */
    if (path.empty())
        std::cout << "No path found\n";
    else
        std::cout << "Path found with " << path.size() << " waypoints\n";

    /* --------------------------------------------------------------------
      6. Wait so we can see the figure
    -------------------------------------------------------------------- */
    cv::waitKey(0);
}
