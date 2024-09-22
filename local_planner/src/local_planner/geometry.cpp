#include "local_planner/geometry.hpp"

namespace local_planning
{

namespace geometry
{

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

std::vector<Point> discretize_circle(Circle circle, double step, Side side)
{
    std::vector<Point> points;
    // get n of points
    int n = 2 * M_PI * circle.radius / step;
    if (side == LEFT)
	{
		for (int i = 0; i < n; ++i)
		{
			Point point;
			point.x = circle.center.x + circle.radius  * std::cos((((2.0 * M_PI) / n) * i) - M_PI_2);
			point.y = circle.center.y + circle.radius  * std::sin((((2.0 * M_PI) / n) * i) - M_PI_2);
			points.push_back(point);
		}
	}
	else
	{
		for (size_t i = 0; i < n; i ++)
		{
			Point point;
			point.x = circle.center.x + circle.radius  * std::cos((((2.0 * M_PI) / n) * i) + M_PI_2);
			point.y = circle.center.y + circle.radius  * std::sin((((2.0 * M_PI) / n) * i) + M_PI_2);
			points.push_back(point);
		}

		std::reverse(points.begin(),
					 points.end());
	}
    return points;
}

double path_length(const std::vector<Point> &path)
{
	double length = 0.0;
	for (size_t i = 1; i < path.size(); i++)
	{
		length += point_distance(path[i - 1], path[i]);
	}
	return length;
}

} // namespace geometry
} // namespace local_planning
