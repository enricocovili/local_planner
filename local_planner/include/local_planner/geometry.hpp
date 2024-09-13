#ifndef GEOMETRY_LOCAL_PLANNER_HPP
#define GEOMETRY_LOCAL_PLANNER_HPP

#include <geometry_msgs/msg/point.hpp>

namespace local_planning
{
    enum ConeColor
    {
        YELLOW,
        BLUE
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
        Line() : m(0.0), c(0.0) {}
    };

    struct Circle
    {
        Point center;
        double radius;

        Circle(Point center, double radius) : center(center), radius(radius) {}
        Circle() : center(Point()), radius(0.0) {}
    };
} // namespace local_planning

#endif // GEOMETRY_LOCAL_PLANNER_HPP