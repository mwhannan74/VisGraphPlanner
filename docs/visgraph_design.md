# Visibility Graph Planner – C++ Design Document

## Objective

Implement a traditional visibility graph (VG) for static 2‑D polygonal environments. Obstacles are known and fixed. Start **S** and goal **G** may change between queries; they shall be inserted after the obstacle‑only VG is constructed.

## Dependencies

* \[Eigen 3] single‑header linear algebra library (fixed‑size `Vector2d`).

No other third‑party code.

## Coordinate type

```cpp
using Point = Eigen::Vector2d; // (x,y)
```

Coordinates are double precision. All algorithms assume counter‑clockwise obstacle vertex order.

## Data Model

| Entity             | Representation                                                                                                    |
| ------------------ | ----------------------------------------------------------------------------------------------------------------- |
| **Obstacle**       | `using Poly = std::vector<Point>;`                                                                                |
| **All vertices**   | `std::vector<Point> vertices_;` — obstacle vertices first, followed by any terminals (S,G, additional waypoints). |
| **Edge**           | `struct Edge { std::size_t to; double cost; };`                                                                   |
| **Adjacency list** | `std::vector<std::vector<Edge>> adj_;` ‑ parallel to `vertices_`.                                                 |

### Index layout

```
0 … n_obs-1         : obstacle vertices
n_obs               : S   (set after build)
n_obs + 1           : G   (set after build)
n_obs + 2 …         : any further terminals
```

## Public API

```cpp
class VisibilityGraph {
public:
    /// Construct from polygonal obstacles.
    explicit VisibilityGraph(const std::vector<Poly>& obstacles);

    /// Insert a terminal (S, G, etc.). Returns index in graph.
    std::size_t addTerminal(const Point& p);

    /// Query shortest path between two vertex indices (Dijkstra).
    std::vector<std::size_t> shortestPath(std::size_t src,
                                          std::size_t dst) const;

    /// Accessors
    const std::vector<Point>&  vertices() const noexcept;
    const std::vector<std::vector<Edge>>& adjacency() const noexcept;

private:
    // geometry helpers
    static bool segmentsIntersect(const Point& a, const Point& b,
                                  const Point& c, const Point& d) noexcept;
    bool isVisible(std::size_t i, std::size_t j) const noexcept;

    void buildObstaclePart();          // O(n³) naïve build
    void connectVertex(std::size_t k); // connect one new vertex

    std::vector<Point> vertices_;
    std::vector<std::vector<Edge>> adj_;
    std::vector<std::pair<Point,Point>> obstacleEdges_;
};
```

## Build procedure

1. **Input stage**
   *Flattens obstacle list:*

   ```cpp
   for (const auto& poly : obstacles)
       for (auto p : poly) vertices_.push_back(p);
   buildObstaclePart();  // uses vertices_[0 .. n_obs-1]
   ```
2. **`buildObstaclePart()`**

   ```
   for i in [0,n_obs):
       for j in (i+1,n_obs):
           if isVisible(i,j): addUndirectedEdge(i,j);
   ```

   * **Visibility test** iterates every obstacle edge; skips edges incident to *i* or *j*; calls `segmentsIntersect`.
3. **`addTerminal(p)`**
   *Append `p`*, push empty adjacency slot, call `connectVertex(k)` which tests visibility only against existing vertices (obstacle + previous terminals) and inserts edges.

Cost:

* Build obstacle part **O(V³)** with V = obstacle vertices.
* Adding each terminal **O(V²)**.

## Geometry primitives

| Function                    | Formula                                                   |
| --------------------------- | --------------------------------------------------------- |
| `orient(p,q,r)`             | `((q-p).x*(r-p).y - (q-p).y*(r-p).x)`                     |
| Proper segment intersection | Two straddling orientation tests plus collinearity check. |

Double‑precision arithmetic plus epsilon threshold (e.g., 1e‑12) accepted; exact predicates could replace later.

## Shortest‑path search

* Dijkstra with binary heap (`std::priority_queue`) over `adj_`.
  Complexity **O(E log V)**; acceptable for ≤10 000 vertices.

Extensibility: replace with A\* (`|p_current – goal|` heuristic) when **G** is known.

## Thread safety

Instance methods except `addTerminal` are `const` and thus thread‑safe. Terminals must not be added concurrently.

## Future extensions (not in scope for MVP)

* Obstacle inflation (Minkowski sum) upstream of build.
* Plane‑sweep O(V² log V) construction to accelerate build.
* Edge culling (reduced VG).
* Caching of visibility rays to speed `addTerminal`.
* Support for polygon holes.

## Example usage

```cpp
std::vector<Poly> obstacles = loadObstacles();
VisibilityGraph vg{obstacles};

auto sIdx = vg.addTerminal(S);
auto gIdx = vg.addTerminal(G);
auto pathIdx = vg.shortestPath(sIdx, gIdx);

std::vector<Point> path;
for (auto id : pathIdx) path.push_back(vg.vertices()[id]);
```

## Build & tooling

* Requires **C++17** compiler, `-DEIGEN_MPL2_ONLY`.
* No linkage libraries; Eigen included as headers.

---

### Complexity summary

| Operation         | Time       | Space       |
| ----------------- | ---------- | ----------- |
| Build obstacle VG | O(N³)      | O(N²) edges |
| Add one terminal  | O(N²)      | O(N) edges  |
| Dijkstra          | O(E log V) | O(V+E)      |

N = # obstacle vertices.

## Acceptance checklist

* [ ] Compiles and links on gcc/clang/MSVC without warnings.
* [ ] Correct path produced in unit tests (convex/concave, no path).
* [ ] Start/goal insertion does not mutate obstacle‑only graph.
* [ ] Public API free of third‑party types except `Eigen::Vector2d`.
