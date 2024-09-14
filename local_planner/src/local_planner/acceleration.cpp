#include "local_planner/acceleration.hpp"

namespace local_planning {

AccelerationPlanner::AccelerationPlanner() : GenericPlanner()
{
    load_params();
    create_connections();
}

void AccelerationPlanner::load_params()
{
    declare_parameter("acceleration.meters_over_horizon", 0.0);
    get_parameter("acceleration.meters_over_horizon", m_meters_over_horizon);

    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "AccelerationPlanner parameters:");
        RCLCPP_INFO(get_logger(), "meters_over_horizon: %f", m_meters_over_horizon);
    }
}

void AccelerationPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    std::vector<Point> y_cones;
    std::vector<Point> b_cones;

    for (size_t i = 0; i < slam_cones->points.size(); ++i)
    {
        if (slam_cones->colors[i].r > 0.9f && slam_cones->colors[i].g > 0.9f) // Yellow
        {
            y_cones.push_back(Point(slam_cones->points[i]));
        }
        if (slam_cones->colors[i].b > 0.9f) // Blue
        {
            b_cones.push_back(Point(slam_cones->points[i]));
        }
    }

    std::array<std::vector<Point>, 2> borders;
    borders[YELLOW] = y_cones;
    borders[BLUE] = b_cones;
    std::vector<Point> center_line = generate_center_line(borders);
    publish_borders(borders);
    publish_center_line(center_line);
}

Line AccelerationPlanner::ransac(std::vector<Point> points)
{
    Line best_line;
    int best_num_inliers = 0;
    for (int i = 0; i < 1000; ++i)
    {
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();
        Point p1 = points[idx1];
        Point p2 = points[idx2];
        Line line(p1, p2);
        int num_inliers = 0;
        for (Point p : points)
        {
            double dist = line.distance_to_point(p);
            if (dist < 0.1)
            {
                num_inliers++;
            }
        }
        if (num_inliers > best_num_inliers)
        {
            best_num_inliers = num_inliers;
            best_line = line;
        }
    }
    return best_line;
}

std::array<std::vector<Point>, 2> AccelerationPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;
    borders[YELLOW] = utils::discretize_line(ransac(y_cones), y_cones.front(), Point(y_cones.back().x, y_cones.back().y+m_meters_over_horizon), m_line_step);
    borders[BLUE] = utils::discretize_line(ransac(b_cones), b_cones.front(), Point(b_cones.back().x, b_cones.back().y+m_meters_over_horizon), m_line_step);
    return borders;
}

std::vector<Point> AccelerationPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    for (size_t i = 0; i < std::min(borders[YELLOW].size(), borders[BLUE].size()); ++i)
    {
        Point p1 = borders[YELLOW][i];
        Point p2 = borders[BLUE][i];
        Point center((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
        center_line.push_back(center);
    }

    return center_line;
}

} // namespace local_planning