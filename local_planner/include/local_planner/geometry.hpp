#ifndef GEOMETRY_LOCAL_PLANNER_HPP
#define GEOMETRY_LOCAL_PLANNER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <cmath>

namespace local_planning
{
namespace geometry
{
    enum ConeColor
    {
        YELLOW,
        BLUE
    };

    enum Side
    {
        LEFT,
        RIGHT
    };

    struct Point
    {
        double x;
        double y;

        Point(double x, double y) : x(x), y(y) {}
        Point(geometry_msgs::msg::Point point) : x(point.x), y(point.y) {}
        Point() : x(0.0), y(0.0) {}
    };

    struct Line
    {
        double m;
        double c;

        Line(double m, double c) : m(m), c(c) {}
        Line(Point p1, Point p2) : m((p2.y - p1.y) / (p2.x - p1.x)), c(p1.y - m * p1.x) {}
        Line() : m(0.0), c(0.0) {}

        double inline distance_to_point(Point p)
        {
            return abs(m * p.x - p.y + c) / std::sqrt(m * m + 1);
        }

        double inline y(double x)
        {
            return m * x + c;
        }
    };

    struct Circle
    {
        Point center;
        double radius;

        Circle(Point center, double radius) : center(center), radius(radius) {}
        Circle() : center(Point()), radius(0.0) {}
    };

    std::vector<Point> discretize_line(Line line, Point start, Point end, double step);
    std::vector<Point> discretize_circle(Circle circle, double step, Side side);
    double inline distance(Point p1, Point p2) { return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)); }

} // namespace geometry
} // namespace local_planning
#endif // GEOMETRY_LOCAL_PLANNER_HPP