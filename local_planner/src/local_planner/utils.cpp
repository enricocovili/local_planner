#include <local_planner/utils.hpp>

namespace local_planning {

namespace utils {

std::vector<Point> discretize_line(Line line, Point start, Point end, double step)
{
    std::vector<Point> points;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double length = std::sqrt(dx * dx + dy * dy);
    int num_points = length / step;
    for (int i = 0; i < num_points; ++i)
    {
        double x = start.x + i * dx / num_points;
        double y = start.y + i * dy / num_points;
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
