/*
 * visibility_graph.hpp – Header-only 2-D visibility graph.
 *
 * ────
 * Core features
 *   • Naïve O(N³) obstacle-only build, query-time insertion of S & G.
 *   • Segment-intersection tests plus “same-polygon chord” rejection.
 *   • Polygon-size validation (skips <3-vertex inputs with a warning).
 *
 * Example (see main_demo.cpp):
 *   vg::VisibilityGraph vg(obstacles);
 *   vg.buildBasic();
 *   auto ids  = vg.injectQueryPts(S,G);
 *   auto path = vg.shortestPath(ids.first, ids.second);
 * ────
 */
#pragma once

#include <algorithm>
#include <cassert>
#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace vg
{
    // geometry and graph component types
    using Point2 = Eigen::Vector2d;
    using Polygon = std::vector<Point2>;   // CCW, simple, ≥3 verts
    struct Segment2 { Point2 a, b; };
    struct Edge { std::size_t to; double cost; };
    struct Vertex { Point2 pos; int poly_id; bool is_query; };

    // Absolute tolerance used by all geometric predicates. Callers should use
    // a coordinate scale for which this fixed tolerance is appropriate.
    inline constexpr double EPS = 1e-12;

    /**
     * Position of a point relative to a simple CCW polygon.
     * Inside: strictly inside the polygon area
     * OnEdge: collinear with, and lying on, an edge or vertex
     * Outside: strictly outside
     */
    enum class PointLocation { Outside, OnEdge, Inside };

    /**
     * @class VisibilityGraph
     * @brief Obstacle visibility graph supporting query-time terminal insertion.
     *
     * Intended lifecycle: construct, call buildBasic(), inject a start/goal
     * pair, then call shortestPath(). Each call to injectQueryPts() replaces
     * the previous query pair while preserving the obstacle-only graph.
     *
     * @note The class provides no internal synchronization. Concurrent
     * read-only access is safe only while no thread is calling buildBasic() or
     * injectQueryPts().
     */
    class VisibilityGraph
    {
    public:
        /**
         * @brief Construct from list of simple CCW polygons.
         *
         * Polygons with <3 vertices are skipped with a warning.
         * Obstacle vertices are copied into @c _vertices in input order.
         * The caller is responsible for supplying finite coordinates and
         * valid, simple, counter-clockwise polygons. Self-intersections,
         * duplicate vertices, and overlapping obstacles are not validated.
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
                    std::cerr << "[VG] Warning: polygon with " << poly.size() << " vertex/vertices ignored (need >=3).\n";
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

            _obstacleVertexCount = _vertices.size();
        }

        /**
         * @brief Build the obstacle-only visibility graph (O(N³)).
         *
         * Recomputes _adjacency from scratch using naive all-pairs
         * visibility checks. Previously injected query vertices are removed.
         */
        void buildBasic()
        {
            _vertices.resize(_obstacleVertexCount);
            const std::size_t n = _obstacleVertexCount;
            _adjacency.assign(n, {});
            for (auto& nbrs : _adjacency)
                nbrs.reserve(6);

            // All-pairs visibility test
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = i + 1; j < n; ++j)
                    if (visible(i, j))
                        addEdge(i, j);

            _obstacleAdjacency = _adjacency;
            _isBuilt = true;
        }


        /**
         * @brief Inject start & goal terminals and connect them.
         *
         * @param S Start position.
         * @param G Goal position.
         * @return Pair of vertex indices (S,G) within the graph.
         *
         * @pre buildBasic() has been called for the obstacle graph.
         * @throws std::logic_error if buildBasic() has not been called.
         * @throws std::runtime_error if S or G is inside an obstacle or on its
         * boundary.
         *
         * Any previous query vertices are removed before the new query is
         * inserted. If S and G are coincident within EPS, one query vertex is
         * inserted and its index is returned for both endpoints.
         *
         * Complexity O(N²) for each inserted point.
         */
        std::pair<std::size_t, std::size_t>
        injectQueryPts(const Point2& S, const Point2& G)
        {
            if (!_isBuilt)
                throw std::logic_error("injectQueryPts: buildBasic() must be called first");

            // Reject points inside or on an obstacle boundary.
            for (const auto& poly : _obstacles)
            {
                if (pointInPolygon(S, poly) != PointLocation::Outside)
                    throw std::runtime_error("injectQueryPts: Start inside or on obstacle");
                if (pointInPolygon(G, poly) != PointLocation::Outside)
                    throw std::runtime_error("injectQueryPts: Goal inside or on obstacle");
            }

            restoreObstacleGraph();

            const std::size_t sid = addVertex(S, /*query=*/true, -1);
            connectQueryVertex(sid);

            if ((S - G).squaredNorm() < EPS * EPS)
                return { sid, sid };

            const std::size_t gid = addVertex(G, /*query=*/true, -1);
            connectQueryVertex(gid);
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
         * @pre s and g are valid vertex indices. Debug builds assert this
         * condition; release builds do not perform a runtime bounds check.
         * @pre The relevant graph edges have been created by buildBasic() and,
         * for query vertices, injectQueryPts().
         *
         * Complexity O(E log V) with binary heap.
         */
        [[nodiscard]]
        std::vector<Point2> shortestPath(std::size_t s,
            std::size_t g) const
        {
            assert(s < _vertices.size() && g < _vertices.size());

            const double INF = std::numeric_limits<double>::infinity();
            const std::size_t N = _vertices.size();

            std::vector<double>       dist(N, INF);
            std::vector<std::size_t>  prev(N, N);

            using Q = std::pair<double, std::size_t>;
            std::priority_queue<Q, std::vector<Q>, std::greater<>> pq;

            dist[s] = 0.0;
            pq.emplace(0.0, s);

            while (!pq.empty())
            {
                const auto [d, u] = pq.top();
                pq.pop();
                if (d > dist[u]) continue;   // stale
                if (u == g) break;           // reached goal

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

            if (dist[g] == INF) return {};   // no path

            std::vector<Point2> path;
            for (auto v = g; v != N; v = prev[v])
                path.push_back(_vertices[v].pos);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Read-only getters
        const std::vector<Polygon>& obstacles() const { return _obstacles; }
        const std::vector<Vertex>& vertices()  const { return _vertices; }
        const std::vector<std::vector<Edge>>& adjacency() const { return _adjacency; }
        
        std::size_t numEdges() const
        {
            std::size_t half = 0;
            for (auto& nbrs : _adjacency) half += nbrs.size();
            return half / 2;
        }

    private:

        // Data
        std::vector<Polygon>              _obstacles; // input polygon for each obstacle
        std::vector<Vertex>               _vertices;  // vertices from each obstacle
        std::vector<std::vector<Edge>>    _adjacency; //  for each vertex, the list of adjacent edges (i.e. the vertices directly visible/connected to it).
        std::vector<std::vector<Edge>>    _obstacleAdjacency;
        std::size_t                       _obstacleVertexCount = 0;
        bool                              _isBuilt = false;

        /**
         * @brief Removes query vertices and restores cached obstacle-only edges.
         */
        void restoreObstacleGraph()
        {
            _vertices.resize(_obstacleVertexCount);
            _adjacency = _obstacleAdjacency;
        }

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
            assert(i < _vertices.size() && j < _vertices.size());

            const double w = (_vertices[i].pos - _vertices[j].pos).norm();
            _adjacency[i].push_back({ j, w });
            _adjacency[j].push_back({ i, w });
        }

        /**
         * @brief Determines if two vertices are mutually visible.
         *
         * Checks whether the line segment between two vertices intersects any obstacle edge.
         * Obstacle edges incident to either candidate endpoint are skipped so
         * a path may meet or follow an obstacle at one of its vertices.
         * Also rejects chords lying entirely within a polygon.
         *
         * @param i Index of the first vertex.
         * @param j Index of the second vertex.
         * @return True if the segment i–j is visible (not blocked by any obstacle), false otherwise.
         */
        bool visible(std::size_t i, std::size_t j) const
        {
            assert(i < _vertices.size() && j < _vertices.size());

            const Segment2 seg{ _vertices[i].pos, _vertices[j].pos };
            if ((seg.a - seg.b).squaredNorm() < EPS * EPS) return false;

            // Skip obstacle edges incident to a candidate endpoint, using EPS
            // rather than exact floating-point equality.
            for (const auto& poly : _obstacles)
            {
                const std::size_t m = poly.size();
                for (std::size_t k = 0; k < m; ++k)
                {
                    const std::size_t k2 = (k + 1) % m;

                    if (((poly[k] - seg.a).squaredNorm() < EPS * EPS) ||
                        ((poly[k] - seg.b).squaredNorm() < EPS * EPS) ||
                        ((poly[k2] - seg.a).squaredNorm() < EPS * EPS) ||
                        ((poly[k2] - seg.b).squaredNorm() < EPS * EPS))
                        continue;

                    if (properIntersection(seg, { poly[k], poly[k2] }))
                        return false;
                }
            }

            // Same-polygon chord passes through interior?
            const int pidA = _vertices[i].poly_id;
            const int pidB = _vertices[j].poly_id;
            if (pidA != -1 && pidA == pidB)
            {
                const Point2 mid = 0.5 * (seg.a + seg.b);
                if (pointInPolygon(mid, _obstacles[pidA]) == PointLocation::Inside)
                    return false;
            }
            return true;
        }

        /**
         * @brief Connects a newly appended query vertex to visible earlier vertices.
         *
         * S is connected to obstacle vertices. G is then connected to obstacle
         * vertices and S, which permits a direct S-G edge when unobstructed.
         *
         * @param q Index of the query vertex to connect.
         */
        void connectQueryVertex(std::size_t q)
        {
            for (std::size_t i = 0; i < q; ++i)
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
         * Handles general and degenerate (collinear) cases using the shared
         * absolute EPS tolerance. Applies orientation tests to detect
         * intersection, including endpoints classified as lying on the other
         * segment.
         *
         * @param s1 First segment.
         * @param s2 Second segment.
         * @return true if the segments intersect (or touch at endpoints),
         *         false otherwise.
         *
         * @note Complexity: O(1)
         */        
        static bool properIntersection(const Segment2& s1, const Segment2& s2)
        {
            if ((s1.a - s1.b).squaredNorm() < EPS * EPS) return false;
            if ((s2.a - s2.b).squaredNorm() < EPS * EPS) return false;

            // Reject disjoint axis-aligned bounding boxes before orientation tests.
            const double min1x = std::min(s1.a.x(), s1.b.x()), max1x = std::max(s1.a.x(), s1.b.x());
            const double min2x = std::min(s2.a.x(), s2.b.x()), max2x = std::max(s2.a.x(), s2.b.x());
            if (max1x < min2x || max2x < min1x) return false;
            const double min1y = std::min(s1.a.y(), s1.b.y()), max1y = std::max(s1.a.y(), s1.b.y());
            const double min2y = std::min(s2.a.y(), s2.b.y()), max2y = std::max(s2.a.y(), s2.b.y());
            if (max1y < min2y || max2y < min1y) return false;

            const double o1 = orient2D(s1.a, s1.b, s2.a);
            const double o2 = orient2D(s1.a, s1.b, s2.b);
            const double o3 = orient2D(s2.a, s2.b, s1.a);
            const double o4 = orient2D(s2.a, s2.b, s1.b);

            const auto haveOppositeSigns = [](double lhs, double rhs)
            {
                return (lhs > EPS && rhs < -EPS) ||
                       (lhs < -EPS && rhs > EPS);
            };

            if (haveOppositeSigns(o1, o2) && haveOppositeSigns(o3, o4)) return true;
            if (std::abs(o1) < EPS && onSegment(s1.a, s1.b, s2.a)) return true;
            if (std::abs(o2) < EPS && onSegment(s1.a, s1.b, s2.b)) return true;
            if (std::abs(o3) < EPS && onSegment(s2.a, s2.b, s1.a)) return true;
            if (std::abs(o4) < EPS && onSegment(s2.a, s2.b, s1.b)) return true;

            return false;
        }

        /**
         * @brief Classifies a point with respect to a polygon (even-odd rule).
         *
         * Implements the crossing-number test. If the point lies on an edge
         * within the absolute EPS tolerance, it is reported as
         * PointLocation::OnEdge; otherwise the usual inside/outside result is
         * returned.
         *
         * @param p    Query point.
         * @param poly Simple, non-self-intersecting polygon in CCW order.
         * @return     PointLocation enum value.
         *
         * Complexity O(n) where n = poly.size().
         */
        static PointLocation pointInPolygon(const Point2& p,
            const Polygon& poly)
        {
            bool inside = false;
            const std::size_t n = poly.size();

            for (std::size_t i = 0, j = n - 1; i < n; j = i++)
            {
                const Point2& a = poly[j];
                const Point2& b = poly[i];

                // Boundary test (collinear and within segment)
                const double o = orient2D(a, b, p);
                if (std::abs(o) < EPS && onSegment(a, b, p))
                    return PointLocation::OnEdge;

                // Ray-casting toggle
                const bool hit = ((a.y() > p.y()) != (b.y() > p.y())) &&
                                 (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y() + EPS) + a.x());
                if (hit) inside = !inside;
            }
            return inside ? PointLocation::Inside : PointLocation::Outside;
        }
    };
} // namespace vg

