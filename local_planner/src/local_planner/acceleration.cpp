#include "local_planner/acceleration.hpp"

namespace local_planning {

AccelerationPlanner::AccelerationPlanner() : GenericPlanner()
{
    load_params();
}

void AccelerationPlanner::load_params()
{
    declare_parameter("acceleration.meters_over_horizon", 0.0);
    get_parameter("acceleration.meters_over_horizon", m_meters_over_horizon);
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

    std::array<std::vector<Point>, 2> borders = generate_borders(y_cones, b_cones);
    std::vector<Point> center_line = generate_center_line(borders);
}

std::array<std::vector<Point>, 2> AccelerationPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    // apply ransac to y_cones and b_cones

    std::array<std::vector<Point>, 2> borders;

    
    return borders;
}

std::vector<Point> AccelerationPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    // take medium points of borders
    for (size_t i = 0; i < std::min(borders[BLUE].size(), borders[YELLOW].size()); ++i)
    {
        center_line.push_back(Point((borders[BLUE][i].x + borders[YELLOW][i].x) / 2.0, (borders[BLUE][i].y + borders[YELLOW][i].y) / 2.0));
    }
    return center_line;
}

} // namespace local_planning