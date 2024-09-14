#include <local_planner/utils.hpp>

namespace local_planning {

namespace utils {

std::vector<Point> discretize_line(Line line, Point start, Point end, double step)
{
    std::vector<Point> points;
    for (double x = start.x; x < end.x; x += step)
    {
        double y = line.y(x);
        points.push_back(Point(x, y));
    }
    return points;
}

std::vector<Point> discretize_circle(Circle circle, double step)
{
    std::vector<Point> points;
    for (double theta = 0; theta < 2 * M_PI; theta += step)
    {
        double x = circle.center.x + circle.radius * std::cos(theta);
        double y = circle.center.y + circle.radius * std::sin(theta);
        points.push_back(Point(x, y));
    }
    return points;
}

} // namespace utils
} // namespace local_planning
