/*
 * visibility_graph.hpp – Header-only 2-D visibility graph with built-in
 * visualisation support (mpocv::Figure).
 *
 * Core features
 *   • Naïve O(N³) obstacle-only build, query-time insertion of S & G.
 *   • Proper intersection tests plus “same-polygon chord” rejection.
 *   • Polygon-size validation (skips <3-vertex inputs with a warning).
 *   • visualize() member renders obstacles, VG edges and the shortest path.
 *
 * Example (see main_demo.cpp):
 *   VisibilityGraph vg(obstacles);
 *   vg.build();
 *   auto ids  = vg.injectQueryPts(S,G);
 *   auto path = vg.shortestPath(ids.first, ids.second);
 *   vg.visualize(S,G,path);
 */

#ifndef VISIBILITY_GRAPH_HPP
#define VISIBILITY_GRAPH_HPP

#include <Eigen/Core>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <cstddef>
#include <utility>
#include <iostream>

#include "figure.h"              // mpocv visualisation dependency

namespace vg
{
    // ─────────── basic geometry types ───────────
    using Point2 = Eigen::Vector2d;
    using Polygon = std::vector<Point2>;   // CCW, simple, ≥3 verts
    struct Segment2 { Point2 a, b; };

    // ─────────── geometry helpers ───────────
    inline double orient2D(const Point2& a, const Point2& b, const Point2& c)
    {
        return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    }

    inline bool onSegment(const Point2& a, const Point2& b, const Point2& p)
    {
        return (p - a).dot(b - p) >= 0.0;
    }

    inline bool properIntersection(const Segment2& s1, const Segment2& s2)
    {
        const double o1 = orient2D(s1.a, s1.b, s2.a), o2 = orient2D(s1.a, s1.b, s2.b);
        const double o3 = orient2D(s2.a, s2.b, s1.a), o4 = orient2D(s2.a, s2.b, s1.b);
        const double eps = 1e-12;
        if ((o1 * o2 < -eps) && (o3 * o4 < -eps)) return true;
        if (std::abs(o1) < eps && onSegment(s1.a, s1.b, s2.a)) return true;
        if (std::abs(o2) < eps && onSegment(s1.a, s1.b, s2.b)) return true;
        if (std::abs(o3) < eps && onSegment(s2.a, s2.b, s1.a)) return true;
        if (std::abs(o4) < eps && onSegment(s2.a, s2.b, s1.b)) return true;
        return false;
    }

    inline bool pointInPolygon(const Point2& p, const Polygon& poly)
    {
        bool inside = false; std::size_t n = poly.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        {
            const Point2& pi = poly[i], & pj = poly[j];
            bool hit = ((pi.y() > p.y()) != (pj.y() > p.y())) &&
                (p.x() < (pj.x() - pi.x()) * (p.y() - pi.y()) / (pj.y() - pi.y() + 1e-18) + pi.x());
            if (hit) inside = !inside;
        }
        return inside;
    }

    // ─────────── graph types ───────────
    struct Edge { std::size_t to; double cost; };
    struct Vertex { Point2 pos; int poly_id; bool is_query; };

    // ─────────── VisibilityGraph ───────────
    class VisibilityGraph
    {
    public:
        explicit VisibilityGraph(const std::vector<Polygon>& obstacles);

        void build();
        std::pair<std::size_t, std::size_t> injectQueryPts(const Point2& S, const Point2& G);
        std::vector<Point2> shortestPath(std::size_t s, std::size_t g) const;

        void visualize(const Point2& start,
            const Point2& goal,
            const std::vector<Point2>& path) const;

        // read-only getters
        const std::vector<Polygon>& obstacles() const { return obstacles_; }
        const std::vector<Vertex>& vertices() const { return vertices_; }
        const std::vector<std::vector<Edge>>& adjacency() const { return adjacency_; }

    private:
        std::size_t addVertex(const Point2& p, bool query, int pid);
        void addEdge(std::size_t i, std::size_t j);
        bool visible(std::size_t i, std::size_t j) const;
        void connectQueryVertex(std::size_t q);

        std::vector<Polygon> obstacles_;
        std::vector<Vertex>  vertices_;
        std::vector<std::vector<Edge>> adjacency_;
    };

    // ─────────── implementation ───────────
    inline VisibilityGraph::VisibilityGraph(const std::vector<Polygon>& obstacles)
    {
        std::size_t reserveN = 0;
        for (const auto& poly : obstacles)
        {
            if (poly.size() < 3)
            {
                std::cerr << "[VG] Warning: polygon with "
                    << poly.size() << " vertex/vertices ignored (need >=3).\n";
                continue;
            }
            obstacles_.push_back(poly);
            reserveN += poly.size();
        }
        vertices_.reserve(reserveN);
        adjacency_.reserve(reserveN);

        for (std::size_t pid = 0; pid < obstacles_.size(); ++pid)
            for (const auto& pt : obstacles_[pid])
                addVertex(pt, false, static_cast<int>(pid));
    }

    inline void VisibilityGraph::build()
    {
        std::size_t n = vertices_.size();
        adjacency_.assign(n, {});
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j)
                if (visible(i, j)) addEdge(i, j);
    }

    inline std::pair<std::size_t, std::size_t>
        VisibilityGraph::injectQueryPts(const Point2& S, const Point2& G)
    {
        auto sid = addVertex(S, true, -1);
        auto gid = addVertex(G, true, -1);
        connectQueryVertex(sid);
        connectQueryVertex(gid);
        return { sid,gid };
    }

    inline std::vector<Point2>
        VisibilityGraph::shortestPath(std::size_t s, std::size_t g) const
    {
        const double INF = std::numeric_limits<double>::infinity();
        std::size_t N = vertices_.size();
        std::vector<double> dist(N, INF);
        std::vector<std::size_t> prev(N, N);

        using Q = std::pair<double, std::size_t>;
        std::priority_queue<Q, std::vector<Q>, std::greater<>> pq;
        dist[s] = 0.0; pq.emplace(0.0, s);

        while (!pq.empty())
        {
            auto [d, u] = pq.top(); pq.pop();
            if (d > dist[u]) continue;
            if (u == g) break;
            for (const auto& e : adjacency_[u])
                if (dist[e.to] > d + e.cost)
                {
                    dist[e.to] = d + e.cost;
                    prev[e.to] = u;
                    pq.emplace(dist[e.to], e.to);
                }
        }
        if (dist[g] == INF) return {};

        std::vector<Point2> path;
        for (auto v = g; v != N; v = prev[v]) path.push_back(vertices_[v].pos);
        std::reverse(path.begin(), path.end());
        return path;
    }

    inline void VisibilityGraph::visualize(const Point2& start,
        const Point2& goal,
        const std::vector<Point2>& path) const
    {
        using namespace mpocv;
        Figure fig(800, 800);

        // polygons
        ShapeStyle st; st.line_color = Color::Blue();
        st.thickness = 1.5f; st.fill_color = Color::Blue(); st.fill_alpha = 0.1f;
        for (const auto& poly : obstacles_)
        {
            std::vector<double> x, y;
            for (const auto& p : poly) { x.push_back(p.x()); y.push_back(p.y()); }
            fig.polygon(x, y, st);
        }

        // edges
        for (std::size_t i = 0; i < adjacency_.size(); ++i)
        {
            const auto& pi = vertices_[i].pos;
            for (const auto& e : adjacency_[i]) if (i < e.to)
            {
                const auto& pj = vertices_[e.to].pos;
                fig.plot({ pi.x(),pj.x() }, { pi.y(),pj.y() }, Color::Black(), 1.0f);
            }
        }

        // path
        if (!path.empty())
        {
            std::vector<double> px, py;
            for (const auto& pt : path) { px.push_back(pt.x()); py.push_back(pt.y()); }
            fig.plot(px, py, Color::Red(), 2.5f, "Path");
        }

        fig.scatter({ start.x() }, { start.y() }, Color::Green(), 6.0f, "Start");
        fig.scatter({ goal.x() }, { goal.y() }, Color::Red(), 6.0f, "Goal");

        fig.grid(true);
        fig.equal_scale(true);
        fig.title("Visibility Graph");
        fig.legend(true);
        fig.show("Visibility Graph");
    }

    // ─────────── private helpers ───────────
    inline std::size_t VisibilityGraph::addVertex(const Point2& p, bool query, int pid)
    {
        vertices_.push_back({ p,pid,query });
        adjacency_.push_back({});
        return vertices_.size() - 1;
    }

    inline void VisibilityGraph::addEdge(std::size_t i, std::size_t j)
    {
        double w = (vertices_[i].pos - vertices_[j].pos).norm();
        adjacency_[i].push_back({ j,w });
        adjacency_[j].push_back({ i,w });
    }

    inline bool VisibilityGraph::visible(std::size_t i, std::size_t j) const
    {
        Segment2 seg{ vertices_[i].pos,vertices_[j].pos };
        if ((seg.a - seg.b).squaredNorm() < 1e-24) return false;

        // edge-edge blocking
        for (const auto& poly : obstacles_)
        {
            std::size_t m = poly.size();
            for (std::size_t k = 0; k < m; ++k)
            {
                std::size_t k2 = (k + 1) % m;
                if ((poly[k] == seg.a || poly[k] == seg.b) ||
                    (poly[k2] == seg.a || poly[k2] == seg.b)) continue;
                if (properIntersection(seg, { poly[k],poly[k2] })) return false;
            }
        }
        // same-polygon chord rejection
        int pidA = vertices_[i].poly_id, pidB = vertices_[j].poly_id;
        if (pidA != -1 && pidA == pidB)
        {
            Point2 mid = 0.5 * (seg.a + seg.b);
            if (pointInPolygon(mid, obstacles_[pidA])) return false;
        }
        return true;
    }

    inline void VisibilityGraph::connectQueryVertex(std::size_t q)
    {
        std::size_t N = vertices_.size();
        for (std::size_t i = 0; i < N - 1; ++i)
            if (visible(i, q)) addEdge(i, q);
    }

} // namespace vg
#endif // VISIBILITY_GRAPH_HPP
