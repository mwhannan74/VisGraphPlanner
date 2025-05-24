// main_demo.cpp – Enhanced demo of the VisibilityGraph class with 20 non‐overlapping obstacles.
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
#include <chrono>
#include <vector>
#include <random>

using namespace vg;

int main()
{
    // 1. Create # non‐overlapping square obstacles in a mxn grid    
    const int rows = 5, cols = 5;
    const double size = 5.0;
    const double gap = 5.0;

    const double noise_frac = 0.5;             // Fraction of gap used as max noise
    const double noise = gap * noise_frac;

    // Set up random number generator for obstacle noise and, later, start/goal offsets
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> noise_dist(-noise, noise);

    std::vector<Polygon> obstacles;
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            // Base grid position
            double x0_base = c * (size + gap);
            double y0_base = r * (size + gap);
            // Add noise jitter
            double x0 = x0_base + noise_dist(gen);
            double y0 = y0_base + noise_dist(gen);
            obstacles.push_back(Polygon{
                { x0,          y0 },
                { x0 + size,   y0 },
                { x0 + size, y0 + size },
                { x0,        y0 + size }
                });
        }
    }

    // 2. Construct and build the visibility graph
    VisibilityGraph vg(obstacles);
    const auto t0 = std::chrono::high_resolution_clock::now();
    vg.buildBasic();
    const auto t1 = std::chrono::high_resolution_clock::now();

    // 3. Define start and goal outside the obstacle field
    double s_dx = 0.0, s_dy = 0.0;
    double g_dx = 0.0, g_dy = 0.0;

    //std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(-2*gap, 2*gap);
    s_dx = dist(gen);
    s_dy = dist(gen);
    g_dx = dist(gen);
    g_dy = dist(gen);

    Point2 S(-1.0 + s_dx, -1.0 + s_dy);
    Point2 G(cols * (size + gap) + 1.0 + g_dx, rows * (size + gap) + 1.0 + g_dy);

    //Point2 S(-1.0, -1.0);
    //Point2 G(cols * (size + gap) + 1.0, rows * (size + gap) + 1.0);

    // 4. Inject S/G and solve for shortest path
    auto [sid, gid] = vg.injectQueryPts(S, G);
    auto path = vg.shortestPath(sid, gid);
    const auto t2 = std::chrono::high_resolution_clock::now();

    // 5. Timing output
    auto build_us = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    auto solve_us = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "buildBasic() took " << build_us << " ms\n";
    std::cout << "shortestPath() took " << solve_us << " ms\n";
    
    std::cout << "Graph has " << vg.numEdges() << " edges\n";

    // 6. Visualise the obstacles, graph, and solution path
    vg.visualize(S, G, path);

    // 7. Console summary
    if (path.empty())
        std::cout << "No path found\n";
    else
        std::cout << "Path found with " << path.size() << " waypoints\n";

    // 8. Wait for key press to close visualization
    cv::waitKey(0);
    return 0;
}
