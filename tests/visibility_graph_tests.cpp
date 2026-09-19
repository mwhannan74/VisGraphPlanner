/*
 * VisibilityGraph tests
 *
 * This is a small, dependency-free test program. It runs every test in the
 * order listed in TEST_CASES and prints PASS or FAIL with the elapsed time.
 *
 * From the repository root:
 *
 *   .\build\Release\visgraph_tests.exe
 *
 * Tests:
 *   - direct path without obstacles
 *   - shortest path around a square
 *   - rejection of diagonals through an obstacle
 *   - rejection of crossings through small-scale obstacles
 *   - rejection of a query inside an obstacle
 *   - ignoring polygons with fewer than three vertices
 *   - identical start and goal
 *   - query on an obstacle edge
 *   - query on an obstacle vertex
 *   - repeated query insertion
 *   - rebuilding after query insertion
 *   - querying before buildBasic()
 *
 * Each test is an ordinary function. Add a new function, use require() to
 * check its result, then add its name and function to TEST_CASES. main() runs
 * the full list and returns a nonzero exit code if any test fails.
 */

#include "visibility_graph.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using vg::Point2;
    using vg::Polygon;
    using vg::VisibilityGraph;

    constexpr double TEST_EPS = 1e-9;

    // Minimal test helpers. A failed requirement throws so the runner can
    // report the failure and continue with the remaining cases.
    class TestFailure : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    void require(bool condition, const std::string& message)
    {
        if (!condition)
            throw TestFailure(message);
    }

    bool pointsNear(const Point2& lhs, const Point2& rhs)
    {
        return (lhs - rhs).norm() <= TEST_EPS;
    }

    double pathLength(const std::vector<Point2>& path)
    {
        double length = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i)
            length += (path[i] - path[i - 1]).norm();
        return length;
    }

    bool hasEdge(const VisibilityGraph& graph, std::size_t from, std::size_t to)
    {
        for (const auto& edge : graph.adjacency().at(from))
            if (edge.to == to)
                return true;
        return false;
    }

    template <typename Exception, typename Function>
    void requireThrows(Function&& function, const std::string& message)
    {
        try
        {
            std::forward<Function>(function)();
        }
        catch (const Exception&)
        {
            return;
        }
        catch (const std::exception& error)
        {
            throw TestFailure(message + "; caught different exception: " + error.what());
        }

        throw TestFailure(message + "; no exception was thrown");
    }

    Polygon makeSquare()
    {
        return {
            Point2(0.0, 0.0),
            Point2(4.0, 0.0),
            Point2(4.0, 4.0),
            Point2(0.0, 4.0)
        };
    }

    // Working behavior and straightforward cases are listed first.

    void directPathInEmptyEnvironment()
    {
        VisibilityGraph graph({});
        graph.buildBasic();

        const Point2 start(-1.0, 2.0);
        const Point2 goal(6.0, 2.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(path.size() == 2, "unobstructed path should contain only start and goal");
        require(pointsNear(path.front(), start), "unobstructed path should start at S");
        require(pointsNear(path.back(), goal), "unobstructed path should end at G");
        require(std::abs(pathLength(path) - 7.0) <= TEST_EPS,
            "unobstructed path should have Euclidean length");
    }

    void shortestPathRoutesAroundSquare()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        const Point2 start(-1.0, 2.0);
        const Point2 goal(5.0, 2.0);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        const double expectedLength = 4.0 + 2.0 * std::sqrt(5.0);
        require(path.size() == 4, "path around a square should use two obstacle corners");
        require(pointsNear(path.front(), start), "square route should start at S");
        require(pointsNear(path.back(), goal), "square route should end at G");
        require(std::abs(pathLength(path) - expectedLength) <= TEST_EPS,
            "square route should have the expected shortest length");
    }

    void obstacleBoundaryEdgesExcludeInteriorChord()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        require(hasEdge(graph, 0, 1), "adjacent polygon vertices should be connected");
        require(hasEdge(graph, 0, 3), "closing polygon edge should be connected");
        require(!hasEdge(graph, 0, 2), "diagonal through polygon interior must be rejected");
        require(!hasEdge(graph, 1, 3), "opposite polygon diagonal must be rejected");
    }

    void smallScaleObstacleBlocksDirectPath()
    {
        const Polygon smallSquare{
            Point2(0.0, 0.0),
            Point2(1e-4, 0.0),
            Point2(1e-4, 1e-4),
            Point2(0.0, 1e-4)
        };
        VisibilityGraph graph({ smallSquare });
        graph.buildBasic();

        const Point2 start(-1e-4, 5e-5);
        const Point2 goal(2e-4, 5e-5);
        const auto [startId, goalId] = graph.injectQueryPts(start, goal);
        const auto path = graph.shortestPath(startId, goalId);

        require(!hasEdge(graph, startId, goalId),
            "a direct edge crossing a small-scale obstacle must be rejected");
        require(path.size() > 2,
            "a path crossing a small-scale obstacle should route around it");
        require(pathLength(path) > (goal - start).norm(),
            "the routed path should be longer than the blocked direct segment");
    }

    void queryInsideObstacleIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(2.0, 2.0), Point2(6.0, 2.0)); },
            "query point inside an obstacle should be rejected");
    }

    void undersizedPolygonIsIgnored()
    {
        const Polygon line{ Point2(0.0, 0.0), Point2(1.0, 0.0) };
        std::ostringstream warning;
        auto* originalErrorBuffer = std::cerr.rdbuf(warning.rdbuf());
        VisibilityGraph graph({ line });
        std::cerr.rdbuf(originalErrorBuffer);
        graph.buildBasic();

        require(graph.obstacles().empty(), "polygon with fewer than three vertices should be ignored");
        require(graph.vertices().empty(), "ignored polygon should not contribute graph vertices");
        require(!warning.str().empty(), "ignored polygon should produce a warning");
    }

    // Less common lifecycle and boundary cases.

    void identicalStartAndGoalReturnsZeroLengthPath()
    {
        VisibilityGraph graph({});
        graph.buildBasic();

        const Point2 query(3.0, -2.0);
        const auto [startId, goalId] = graph.injectQueryPts(query, query);
        const auto path = graph.shortestPath(startId, goalId);

        require(path.size() == 1, "identical start and goal should return one point");
        require(pointsNear(path.front(), query), "zero-length path should contain the query point");
        require(pathLength(path) <= TEST_EPS, "identical start and goal should have zero path length");
    }

    void queryOnEdgeIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(2.0, 0.0), Point2(6.0, 2.0)); },
            "query point on an obstacle edge should be rejected");
    }

    void queryOnVertexIsRejected()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        requireThrows<std::runtime_error>(
            [&graph] { graph.injectQueryPts(Point2(0.0, 0.0), Point2(6.0, 2.0)); },
            "query point on an obstacle vertex should be rejected");
    }

    void repeatedQueriesDoNotAccumulateVertices()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();

        graph.injectQueryPts(Point2(-1.0, 1.0), Point2(5.0, 1.0));
        graph.injectQueryPts(Point2(-1.0, 3.0), Point2(5.0, 3.0));

        require(graph.vertices().size() == makeSquare().size() + 2,
            "a new query should replace the previous start and goal vertices");
    }

    void rebuildRestoresObstacleOnlyGraph()
    {
        VisibilityGraph graph({ makeSquare() });
        graph.buildBasic();
        graph.injectQueryPts(Point2(-1.0, 2.0), Point2(5.0, 2.0));

        graph.buildBasic();

        require(graph.vertices().size() == makeSquare().size(),
            "rebuilding should remove previously injected query vertices");
        require(graph.adjacency().size() == makeSquare().size(),
            "rebuilt adjacency should contain obstacle vertices only");
    }

    void queryBeforeBuildIsNotSilentlyIncomplete()
    {
        VisibilityGraph graph({ makeSquare() });

        try
        {
            const auto [startId, goalId] = graph.injectQueryPts(Point2(-1.0, 2.0), Point2(5.0, 2.0));
            const auto path = graph.shortestPath(startId, goalId);
            require(!path.empty(),
                "query before build must either construct a complete graph or reject the call");
        }
        catch (const std::logic_error&)
        {
            return;
        }
    }

    struct TestCase
    {
        const char* name;
        void (*run)();
    };

    // To add a test, write a function above and add one entry here.
    const std::vector<TestCase> TEST_CASES{
        { "Direct path in empty environment", directPathInEmptyEnvironment },
        { "Shortest path routes around square", shortestPathRoutesAroundSquare },
        { "Obstacle boundary excludes interior chord", obstacleBoundaryEdgesExcludeInteriorChord },
        { "Small-scale obstacle blocks direct path", smallScaleObstacleBlocksDirectPath },
        { "Query inside obstacle is rejected", queryInsideObstacleIsRejected },
        { "Undersized polygon is ignored", undersizedPolygonIsIgnored },
        { "Identical start and goal", identicalStartAndGoalReturnsZeroLengthPath },
        { "Query on edge is rejected", queryOnEdgeIsRejected },
        { "Query on vertex is rejected", queryOnVertexIsRejected },
        { "Repeated queries replace old queries", repeatedQueriesDoNotAccumulateVertices },
        { "Rebuild restores obstacle-only graph", rebuildRestoresObstacleOnlyGraph },
        { "Query before build is not silently incomplete", queryBeforeBuildIsNotSilentlyIncomplete }
    };
}

int main()
{
    using Clock = std::chrono::steady_clock;

    std::size_t failed = 0;
    const auto suiteStart = Clock::now();

    std::cout << "Running " << TEST_CASES.size() << " tests\n\n";

    for (std::size_t index = 0; index < TEST_CASES.size(); ++index)
    {
        const auto& test = TEST_CASES[index];
        const auto testStart = Clock::now();
        std::cout << '[' << index + 1 << '/' << TEST_CASES.size() << "] "
                  << test.name << " ... " << std::flush;

        try
        {
            test.run();
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "PASS (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n";
        }
        catch (const std::exception& error)
        {
            ++failed;
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "FAIL (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n"
                      << "       " << error.what() << '\n';
        }
        catch (...)
        {
            ++failed;
            const auto elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - testStart).count();
            std::cout << "FAIL (" << std::fixed << std::setprecision(2)
                      << elapsed << " ms)\n"
                      << "       Unknown exception\n";
        }
    }

    const auto passed = TEST_CASES.size() - failed;
    const auto elapsed = std::chrono::duration<double, std::milli>(
        Clock::now() - suiteStart).count();

    std::cout << "\nResult: " << passed << " passed, " << failed
              << " failed, " << TEST_CASES.size() << " total ("
              << std::fixed << std::setprecision(2) << elapsed << " ms)\n";
    return failed == 0 ? 0 : 1;
}
