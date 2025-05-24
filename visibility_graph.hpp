/*
 * visibility_graph.hpp – Header-only 2-D visibility graph with built-in
 * visualisation support (mpocv::Figure).
 *
 * ────
 * Core features
 *   • Naïve O(N³) obstacle-only build, query-time insertion of S & G.
 *   • Proper intersection tests plus “same-polygon chord” rejection.
 *   • Polygon-size validation (skips <3-vertex inputs with a warning).
 *   • visualize() member renders obstacles, VG edges and the shortest path.
 *
 * Example (see main_demo.cpp):
 *   vg::VisibilityGraph vg(obstacles);
 *   vg.build();
 *   auto ids  = vg.injectQueryPts(S,G);
 *   auto path = vg.shortestPath(ids.first, ids.second);
 *   vg.visualize(S,G,path);
 * ────
 */
#pragma once

#include <Eigen/Core>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <cstddef>
#include <utility>
#include <iostream>

#include "figure.h" // mpocv visualisation dependency

namespace vg
{
    // geometry and graph component types
    using Point2 = Eigen::Vector2d;
    using Polygon = std::vector<Point2>;   // CCW, simple, ≥3 verts
    struct Segment2 { Point2 a, b; };
    struct Edge { std::size_t to; double cost; };
    struct Vertex { Point2 pos; int poly_id; bool is_query; };


    /**
     * @class VisibilityGraph
     * @brief Obstacle visibility graph supporting query-time terminal insertion.
     *
     * @note All public methods are thread-safe except injectQueryPts().
     */
    class VisibilityGraph
    {
    public:
        /**
         * @brief Construct from list of simple CCW polygons.
         *
         * Polygons with <3 vertices are skipped with a warning.
         * Obstacle vertices are copied into @c vertices_ in input order.
         *
         * @param obstacles List of polygons representing obstacles.
         */
        explicit VisibilityGraph(const std::vector<Polygon>& obstacles)
        {
            std::size_t reserveN = 0;

            // Validate and cache obstacles
            for (const auto& poly : obstacles)
            {
                if (poly.size() < 3)
                {
                    std::cerr << "[VG] Warning: polygon with "
                        << poly.size()
                        << " vertex/vertices ignored (need >=3).\n";
                    continue;
                }
                _obstacles.push_back(poly);
                reserveN += poly.size();
            }

            // Pre-allocate storage
            _vertices.reserve(reserveN);
            _adjacency.reserve(reserveN);

            // Flatten obstacle vertices
            for (std::size_t pid = 0; pid < _obstacles.size(); ++pid)
                for (const auto& pt : _obstacles[pid])
                    addVertex(pt, /*query=*/false, static_cast<int>(pid));
        }

        /**
         * @brief Build the obstacle-only visibility graph (O(N³)).
         *
         * Recomputes @c adjacency_ from scratch using naive all-pairs
         * visibility checks.
         *
         * @warning Call only once after construction; subsequent calls will
         *          override any previously injected query vertices.
         */
        void build()
        {
            const std::size_t n = _vertices.size();
            _adjacency.assign(n, {});

            // All-pairs visibility test
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = i + 1; j < n; ++j)
                    if (visible(i, j))
                        addEdge(i, j);
        }

        /**
         * @brief Inject start & goal terminals and connect them.
         *
         * @param S Start position.
         * @param G Goal position.
         * @return Pair of vertex indices (S,G) within the graph.
         *
         * Complexity O(N²) for each inserted point.
         */
        std::pair<std::size_t, std::size_t>
            injectQueryPts(const Point2& S, const Point2& G)
        {
            const std::size_t sid = addVertex(S, /*query=*/true, -1);
            const std::size_t gid = addVertex(G, /*query=*/true, -1);

            connectQueryVertex(sid);   // connect S → obstacles
            connectQueryVertex(gid);   // connect G → obstacles
            return { sid, gid };
        }

        /**
         * @brief Dijkstra shortest path in visibility graph.
         *
         * @param s Source vertex index.
         * @param g Target vertex index.
         * @return Sequence of points from @p s to @p g (inclusive); empty if
         *         no path exists.
         *
         * Complexity O(E log V) with binary heap.
         */
        std::vector<Point2> shortestPath(std::size_t s,
            std::size_t g) const
        {
            const double INF = std::numeric_limits<double>::infinity();
            const std::size_t N = _vertices.size();

            std::vector<double>       dist(N, INF);
            std::vector<std::size_t>  prev(N, N);

            // Min-heap of (distance, vertex)
            using Q = std::pair<double, std::size_t>;
            std::priority_queue<Q, std::vector<Q>, std::greater<>> pq;

            dist[s] = 0.0;
            pq.emplace(0.0, s);

            while (!pq.empty())
            {
                const auto [d, u] = pq.top();
                pq.pop();
                if (d > dist[u]) continue;   // Stale entry
                if (u == g) break;           // Early exit

                for (const auto& e : _adjacency[u])
                {
                    const double alt = d + e.cost;
                    if (alt < dist[e.to])
                    {
                        dist[e.to] = alt;
                        prev[e.to] = u;
                        pq.emplace(alt, e.to);
                    }
                }
            }

            if (dist[g] == INF) return {};   // No path

            // Reconstruct path (reverse)
            std::vector<Point2> path;
            for (auto v = g; v != N; v = prev[v])
                path.push_back(_vertices[v].pos);
            std::reverse(path.begin(), path.end());
            return path;
        }

        /**
         * @brief Convenience OpenCV plot of obstacles, graph, and path.
         *
         * @param start Start point (for marker).
         * @param goal  Goal point  (for marker).
         * @param path  Polyline returned from shortestPath().
         *
         * Render order: polygons -> graph edges -> path -> terminals.
         */
        void visualize(const Point2& start,
            const Point2& goal,
            const std::vector<Point2>& path) const
        {
            using namespace mpocv;
            Figure fig(800, 800);

            //  draw polygons 
            ShapeStyle st;
            st.line_color = Color::Blue();
            st.thickness = 1.5f;
            st.fill_color = Color::Blue();
            st.fill_alpha = 0.1f;

            for (const auto& poly : _obstacles)
            {
                std::vector<double> x, y;
                for (const auto& p : poly)
                {
                    x.push_back(p.x());
                    y.push_back(p.y());
                }
                fig.polygon(x, y, st);
            }

            //  draw graph edges 
            for (std::size_t i = 0; i < _adjacency.size(); ++i)
            {
                const auto& pi = _vertices[i].pos;
                for (const auto& e : _adjacency[i])
                {
                    if (i >= e.to) continue; // Draw each undirected edge once
                    const auto& pj = _vertices[e.to].pos;
                    fig.plot({ pi.x(), pj.x() },
                        { pi.y(), pj.y() },
                        Color::Black(), 1.0f);
                }
            }

            //  draw shortest path 
            if (!path.empty())
            {
                std::vector<double> px, py;
                for (const auto& pt : path)
                {
                    px.push_back(pt.x());
                    py.push_back(pt.y());
                }
                fig.plot(px, py, Color::Red(), 2.5f, "Path");
            }

            // Terminals
            fig.scatter({ start.x() }, { start.y() },
                Color::Green(), 6.0f, "Start");
            fig.scatter({ goal.x() }, { goal.y() },
                Color::Red(), 6.0f, "Goal");

            fig.grid(true);
            fig.equal_scale(true);
            fig.title("Visibility Graph");
            fig.legend(true);
            fig.show("Visibility Graph");
        }

        // Read-only getters
        const std::vector<Polygon>& obstacles() const { return _obstacles; }
        const std::vector<Vertex>& vertices()  const { return _vertices; }
        const std::vector<std::vector<Edge>>& adjacency() const { return _adjacency; }

    private:

        // Data
        std::vector<Polygon>              _obstacles;
        std::vector<Vertex>               _vertices;
        std::vector<std::vector<Edge>>    _adjacency;

        /**
         * @brief Adds a new vertex to the graph.
         *
         * This method appends a vertex to the internal vertex list and initializes
         * an empty adjacency list entry for it.
         *
         * @param p      The 2D point position of the vertex.
         * @param query  True if this vertex is a query point (e.g. start/goal), false if part of an obstacle.
         * @param pid    Polygon ID this vertex belongs to; -1 for query vertices.
         * @return The index of the newly added vertex.
         */
        std::size_t addVertex(const Point2& p, bool query, int pid)
        {
            _vertices.push_back({ p, pid, query });
            _adjacency.push_back({});
            return _vertices.size() - 1;
        }

        /**
         * @brief Adds an undirected edge between two existing vertices.
         *
         * The edge weight is the Euclidean distance between the two vertices.
         * Adds the edge in both directions (i → j and j → i).
         *
         * @param i Index of the first vertex.
         * @param j Index of the second vertex.
         */
        void addEdge(std::size_t i, std::size_t j)
        {
            const double w = (_vertices[i].pos - _vertices[j].pos).norm();
            _adjacency[i].push_back({ j, w });
            _adjacency[j].push_back({ i, w });
        }

        /**
         * @brief Determines if two vertices are mutually visible.
         *
         * Checks whether the line segment between two vertices intersects any obstacle edge.
         * Skips segments that overlap with existing polygon edges.
         * Also rejects chords lying entirely within a polygon.
         *
         * @param i Index of the first vertex.
         * @param j Index of the second vertex.
         * @return True if the segment i–j is visible (not blocked by any obstacle), false otherwise.
         */
        bool visible(std::size_t i, std::size_t j) const
        {
            const Segment2 seg{ _vertices[i].pos, _vertices[j].pos };

            if ((seg.a - seg.b).squaredNorm() < 1e-24) return false;

            for (const auto& poly : _obstacles)
            {
                const std::size_t m = poly.size();
                for (std::size_t k = 0; k < m; ++k)
                {
                    const std::size_t k2 = (k + 1) % m;

                    if ((poly[k] == seg.a) || (poly[k] == seg.b) ||
                        (poly[k2] == seg.a) || (poly[k2] == seg.b))
                        continue;

                    if (properIntersection(seg, { poly[k], poly[k2] }))
                        return false;
                }
            }

            const int pidA = _vertices[i].poly_id;
            const int pidB = _vertices[j].poly_id;
            if (pidA != -1 && pidA == pidB)
            {
                const Point2 mid = 0.5 * (seg.a + seg.b);
                if (pointInPolygon(mid, _obstacles[pidA]))
                    return false;
            }
            return true;
        }

        /**
         * @brief Connects a query vertex to all visible non-query vertices.
         *
         * Iterates over all previously added vertices (excluding other query points),
         * and adds an edge between the query vertex and each visible vertex.
         *
         * @param q Index of the query vertex to connect.
         */
        void connectQueryVertex(std::size_t q)
        {
            const std::size_t N = _vertices.size();
            for (std::size_t i = 0; i < N - 1; ++i)
                if (visible(i, q))
                    addEdge(i, q);
        }


        /**
         * @brief Computes twice the signed area of the triangle (a, b, c).
         *
         * Used for orientation testing in 2D geometry. The result indicates
         * the relative orientation of the three points:
         *   - Positive: counter-clockwise (CCW)
         *   - Negative: clockwise (CW)
         *   - Zero: collinear
         *
         * @param a First point.
         * @param b Second point.
         * @param c Third point.
         * @return A scalar value proportional to the signed area of the triangle.
         */
        static double orient2D(const Point2& a, const Point2& b, const Point2& c)
        {
            return (b.x() - a.x()) * (c.y() - a.y()) -
                (b.y() - a.y()) * (c.x() - a.x());
        }

        /**
         * @brief Checks whether a point lies on a closed line segment.
         *
         * Assumes the point is collinear with the segment. Uses dot product
         * to determine if the point lies between endpoints a and b.
         *
         * @param a One endpoint of the segment.
         * @param b The other endpoint of the segment.
         * @param p The point to test.
         * @return true if point p lies on the segment [a, b], false otherwise.
         */
        static bool onSegment(const Point2& a, const Point2& b, const Point2& p)
        {
            return (p - a).dot(b - p) >= 0.0;
        }

        /**
         * @brief Tests whether two 2D line segments intersect or touch.
         *
         * Handles general and degenerate (collinear) cases robustly.
         * Applies orientation tests to detect intersection, including
         * when endpoints lie exactly on the other segment.
         *
         * @param s1 First segment.
         * @param s2 Second segment.
         * @return true if the segments intersect (or touch at endpoints),
         *         false otherwise.
         *
         * @note Numerical stability is controlled with an epsilon threshold.
         * @note Complexity: O(1)
         */
        static bool properIntersection(const Segment2& s1, const Segment2& s2)
        {
            const double o1 = orient2D(s1.a, s1.b, s2.a);
            const double o2 = orient2D(s1.a, s1.b, s2.b);
            const double o3 = orient2D(s2.a, s2.b, s1.a);
            const double o4 = orient2D(s2.a, s2.b, s1.b);
            constexpr double eps = 1e-12;

            if ((o1 * o2 < -eps) && (o3 * o4 < -eps)) return true;
            if (std::abs(o1) < eps && onSegment(s1.a, s1.b, s2.a)) return true;
            if (std::abs(o2) < eps && onSegment(s1.a, s1.b, s2.b)) return true;
            if (std::abs(o3) < eps && onSegment(s2.a, s2.b, s1.a)) return true;
            if (std::abs(o4) < eps && onSegment(s2.a, s2.b, s1.b)) return true;
            return false;
        }

        /**
         * @brief Determines whether a 2D point lies inside a polygon.
         *
         * Implements the even-odd (crossing number) rule. Counts the number of
         * edge crossings along a ray extending from the point horizontally.
         * Odd crossings mean the point is inside.
         *
         * @param p The query point.
         * @param poly A simple (non-intersecting), CCW-ordered polygon.
         * @return true if the point lies strictly inside the polygon,
         *         false if outside or on the edge.
         *
         * @note Uses a small epsilon (1e-18) to guard against division-by-zero.
         * @note Complexity: O(n), where n = poly.size().
         */
        static bool pointInPolygon(const Point2& p, const Polygon& poly)
        {
            bool inside = false;
            const std::size_t n = poly.size();
            for (std::size_t i = 0, j = n - 1; i < n; j = i++)
            {
                const Point2& pi = poly[i];
                const Point2& pj = poly[j];
                const bool hit = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                    (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) /
                        (pj.y() - pi.y() + 1e-18) +
                        pi.x());
                if (hit) inside = !inside;
            }
            return inside;
        }

    };

} // namespace vg

