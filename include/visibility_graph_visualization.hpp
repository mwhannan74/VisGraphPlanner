#pragma once

#include "visibility_graph.hpp"
#include "figure.h"

namespace vg
{
    inline void visualize(const VisibilityGraph& graph,
        const Point2& start,
        const Point2& goal,
        const std::vector<Point2>& path = {},
        int pixelSize = 1200)
    {
        using namespace mpocv;

        Figure fig(pixelSize, pixelSize);

        ShapeStyle obstacleStyle;
        obstacleStyle.line_color = Color::Blue();
        obstacleStyle.thickness = 1.5f;
        obstacleStyle.fill_color = Color::Blue();
        obstacleStyle.fill_alpha = 0.1f;

        for (const auto& poly : graph.obstacles())
        {
            std::vector<double> x;
            std::vector<double> y;
            x.reserve(poly.size());
            y.reserve(poly.size());

            for (const auto& p : poly)
            {
                x.push_back(p.x());
                y.push_back(p.y());
            }

            fig.polygon(x, y, obstacleStyle);
        }

        const auto& adjacency = graph.adjacency();
        const auto& vertices = graph.vertices();
        for (std::size_t i = 0; i < adjacency.size(); ++i)
        {
            const auto& pi = vertices[i].pos;
            for (const auto& e : adjacency[i])
            {
                if (i >= e.to) continue;

                const auto& pj = vertices[e.to].pos;
                fig.plot({ pi.x(), pj.x() }, { pi.y(), pj.y() }, Color::Black(), 1.0f);
            }
        }

        if (!path.empty())
        {
            std::vector<double> px;
            std::vector<double> py;
            px.reserve(path.size());
            py.reserve(path.size());

            for (const auto& pt : path)
            {
                px.push_back(pt.x());
                py.push_back(pt.y());
            }

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
}
