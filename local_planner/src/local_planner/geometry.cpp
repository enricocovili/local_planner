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

// Function to interpolate a point using Catmull-Rom spline
Point catmull_rom_spline(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) 
{
    double t2 = t * t;
    double t3 = t2 * t;

    double x = 0.5 * ((2.0 * p1.x) +
                      (-p0.x + p2.x) * t +
                      (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                      (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

    double y = 0.5 * ((2.0 * p1.y) +
                      (-p0.y + p2.y) * t +
                      (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                      (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

    return {x, y};
}

// Function to smooth the points using Catmull-Rom splines
std::vector<Point> smooth_points(const std::vector<Point>& points, int segments) 
{
    std::vector<Point> smoothPath;

    if (points.size() < 4) {
        return smoothPath;
    }

    // Iterate through each set of 4 consecutive points
    for (size_t i = 0; i < points.size() - 3; ++i) {
        const Point& p0 = points[i];
        const Point& p1 = points[i + 1];
        const Point& p2 = points[i + 2];
        const Point& p3 = points[i + 3];

        // Generate intermediate points between p1 and p2
        for (int j = 0; j <= segments; ++j) {
            double t = static_cast<double>(j) / segments;
            smoothPath.push_back(catmull_rom_spline(p0, p1, p2, p3, t));
        }
    }

    return smoothPath;
}

} // namespace geometry
} // namespace local_planning
