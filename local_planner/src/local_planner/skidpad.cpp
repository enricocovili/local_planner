#include "local_planner/skidpad.hpp"

namespace local_planning {

SkidpadPlanner::SkidpadPlanner() : GenericPlanner()
{
    load_params();
}

void SkidpadPlanner::load_params()
{
    declare_parameter("skidpad.center_x", 15.0);
    declare_parameter("skidpad.center_y", 0.0);
    declare_parameter("skidpad.inner_radius", 7.625);
    declare_parameter("skidpad.outer_radius", 10.625);
    declare_parameter("skidpad.max_distance_from_center", 0.0);
    declare_parameter("skidpad.radius_offset", 0.0);
    declare_parameter("skidpad.end_x", 100.0);
    declare_parameter("skidpad.minimum_cones", 3);

    get_parameter("skidpad.center_x", m_center_x);
    get_parameter("skidpad.center_y", m_center_y);
    get_parameter("skidpad.inner_radius", m_inner_radius);
    get_parameter("skidpad.outer_radius", m_outer_radius);
    get_parameter("skidpad.max_distance_from_center", m_max_distance_from_center);
    get_parameter("skidpad.radius_offset", m_radius_offset);
    get_parameter("skidpad.end_x", m_end_x);
    get_parameter("skidpad.minimum_cones", m_minimum_cones);

    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "Skidpad parameters:");
        RCLCPP_INFO(get_logger(), "Center: (%f, %f)", m_center_x, m_center_y);
        RCLCPP_INFO(get_logger(), "Inner radius: %f", m_inner_radius);
        RCLCPP_INFO(get_logger(), "Outer radius: %f", m_outer_radius);
        RCLCPP_INFO(get_logger(), "Max distance from center: %f", m_max_distance_from_center);
        RCLCPP_INFO(get_logger(), "Radius offset: %f", m_radius_offset);
        RCLCPP_INFO(get_logger(), "End x: %f", m_end_x);
        RCLCPP_INFO(get_logger(), "Minimum cones: %d", m_minimum_cones);
    }
}

void SkidpadPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    if (this->m_idle)
    {
        return;
    }

    std::array<std::vector<Point>, 2> left_cones;
    std::array<std::vector<Point>, 2> right_cones;

    for (size_t i = 0; i < slam_cones->points.size(); ++i)
    {
        ConeColor color = slam_cones->colors[i].b > 0.9f ? BLUE : YELLOW;
        if (slam_cones->points[i].y > 0)
        {
            left_cones[color].push_back(slam_cones->points[i]);
        }
        else
        {
            right_cones[color].push_back(slam_cones->points[i]);
        }
    }

    if (left_cones[YELLOW].size() < m_minimum_cones || right_cones[BLUE].size() < m_minimum_cones)
    {
        if (m_debug)
        {
            RCLCPP_INFO(get_logger(), "Not enough cones detected");
            RCLCPP_INFO(get_logger(), "Left yellow: %d, Right blue: %d", left_cones[YELLOW].size(), right_cones[BLUE].size());
        }
        return;
    }

    Circle outer_left = best_circle(left_cones[YELLOW], LEFT);
    Circle outer_right = best_circle(right_cones[BLUE], RIGHT);
    outer_left.radius = m_outer_radius;
    outer_right.radius = m_outer_radius;

    if (distance(outer_left.center, Point(m_center_x, m_center_y)) > m_max_distance_from_center ||
        distance(outer_right.center, Point(m_center_x, -m_center_y)) > m_max_distance_from_center)
    {
        if (m_debug)
        {
            RCLCPP_INFO(get_logger(), "Circles not found or too far from center");
            RCLCPP_INFO(get_logger(), "Distances: %f, %f", distance(outer_left.center, Point(m_center_x, m_center_y)), distance(outer_right.center, Point(m_center_x, -m_center_y)));
        }
        return;
    }

    // outer_left.center.x = (outer_left.center.x + outer_right.center.x) / 2;
    // outer_right.center.x = outer_left.center.x;

    Circle inner_left = Circle(outer_left.center, m_inner_radius);
    Circle inner_right = Circle(outer_right.center, m_inner_radius);
    
    publish_borders(generate_borders(discretize_circle(outer_left, m_circle_step, LEFT), discretize_circle(outer_right, m_circle_step, RIGHT)));
    publish_borders_completed(generate_borders(discretize_circle(inner_left, m_circle_step, LEFT), discretize_circle(inner_right, m_circle_step, RIGHT)));

    publish_center_line(generate_center_line({outer_left, outer_right}));
    publish_center_line_completed(generate_center_line({outer_left, outer_right}));

    this->m_idle = true;
}

Circle SkidpadPlanner::best_circle(std::vector<Point> points, Side side)
{
    Circle result_circle;
    
    for (size_t i = 0; i < points.size(); ++i)
    {
        for (size_t j = i + 1; j < points.size(); ++j)
        {
            for (size_t k = j + 1; k < points.size(); ++k)
            {
                Circle circle = get_circle(points[i], points[j], points[k]);
                if (side == LEFT)
                {
                    if (circle.center.x < m_center_x - m_max_distance_from_center ||
                        circle.center.x > m_center_x + m_max_distance_from_center ||
                        circle.center.y < m_center_y - m_max_distance_from_center ||
                        circle.center.y > m_center_y + m_max_distance_from_center)
                    {
                        continue;
                    }
                    if (distance(circle.center, Point(m_center_x, m_center_y)) < distance(result_circle.center, Point(m_center_x, m_center_y)))
                    {
                        result_circle = circle;
                    }
                }
                else
                {
                    if (circle.center.x < m_center_x - m_max_distance_from_center ||
                        circle.center.x > m_center_x + m_max_distance_from_center ||
                        circle.center.y > -m_center_y + m_max_distance_from_center ||
                        circle.center.y < -m_center_y - m_max_distance_from_center)
                    {
                        continue;
                    }
                    if (distance(circle.center, Point(m_center_x, -m_center_y)) < distance(result_circle.center, Point(m_center_x, -m_center_y)))
                    {
                        result_circle = circle;
                    }
                }
            }
        }
    }

    return result_circle;
}

Circle SkidpadPlanner::get_circle(Point p1, Point p2, Point p3)
{
    Circle circle;

	double d = 2.0 * (p1.x * (p2.y - p3.y) +
					  p2.x * (p3.y - p1.y) +
					  p3.x * (p1.y - p2.y));

	circle.center.x = ((p1.x * p1.x + p1.y * p1.y) * (p2.y - p3.y) +
				(p2.x * p2.x + p2.y * p2.y) * (p3.y - p1.y) +
				(p3.x * p3.x + p3.y * p3.y) * (p1.y - p2.y)) / d;

	circle.center.y = ((p1.x * p1.x + p1.y * p1.y) * (p3.x - p2.x) +
				(p2.x * p2.x + p2.y * p2.y) * (p1.x - p3.x) +
				(p3.x * p3.x + p3.y * p3.y) * (p2.x - p1.x)) / d;

    return circle;   
}

std::array<std::vector<Point>, 2> SkidpadPlanner::generate_borders(std::vector<Point> inner, std::vector<Point> outer)
{
    return {inner, outer};
}

std::vector<Point> SkidpadPlanner::generate_center_line(std::array<Circle, 2> borders)
{
    std::vector<Point> center_line;
    double target_y = (borders[LEFT].center.y+borders[RIGHT].center.y)/2;
    Point start = Point(0, 0);
    Point end = Point(borders[LEFT].center.x, target_y);
    // step 1: line from 0,0 to borders[0].center.x, borders[0].center.y
    std::vector<Point> line = discretize_line(Line(start, end), start, end, m_line_step);
    center_line.insert(center_line.end(), line.begin(), line.end());
    // step 2: right circle 2 times and left circle 2 times
    for (auto i: {RIGHT, RIGHT, LEFT, LEFT})
    {
        Circle center_circle = borders[i];
        center_circle.radius = (m_inner_radius + m_outer_radius) / 2 + m_radius_offset;
        std::vector<Point> circle = discretize_circle(center_circle, m_circle_step, i);
        center_line.insert(center_line.end(), circle.begin(), circle.end());
    }

    start = Point(borders[LEFT].center.x, target_y);
    end = Point(m_end_x, target_y);
    // step 3: line from borders[1].center.x, borders[1].center.y to m_end_x, 0
    line = discretize_line(Line(start, end), start, end, m_line_step);

    center_line.insert(center_line.end(), line.begin(), line.end());

    return center_line;
}

} // namespace local_plannings