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
    declare_parameter("acceleration.safe_slope", 0.0);

    get_parameter("acceleration.meters_over_horizon", m_meters_over_horizon);
    get_parameter("acceleration.safe_slope", m_safe_slope);
    
    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "AccelerationPlanner parameters:");
        RCLCPP_INFO(get_logger(), "meters_over_horizon: %f", m_meters_over_horizon);
    }
}

void AccelerationPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    if (m_idle)
    {
        return;
    }
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

    if (std::abs(Line(center_line.front(), center_line.back()).m) < m_safe_slope)
    {
        publish_center_line_completed(center_line);
        publish_borders_completed(borders);
        m_idle = true;
    }
    else
    {
        publish_center_line(center_line);
        publish_borders(borders);
    }
}

// @brief: Least squares method to fit a line to a set of points
// @param: points: vector of points to fit the line to
// @return: Line object representing the best fit line
Line AccelerationPlanner::best_fit_line(std::vector<Point> points)
{
    size_t n = points.size();
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    
    // Compute the necessary summations
    for (const auto& point : points) {
        sumX += point.x;
        sumY += point.y;
        sumXY += point.x * point.y;
        sumX2 += point.x * point.x;
    }
    
    // Compute the slope (m) and intercept (c)
    double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    double c = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
    return Line(m, c);
}

std::array<std::vector<Point>, 2> AccelerationPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;

    Line best_y_line = best_fit_line(y_cones);
    borders[YELLOW] = utils::discretize_line(best_y_line, Point(0, best_y_line.y(0)), Point(m_meters_over_horizon, best_y_line.y(m_meters_over_horizon)), m_line_step);
    
    Line best_b_line = best_fit_line(b_cones);
    borders[BLUE] = utils::discretize_line(best_b_line, Point(0, best_b_line.y(0)), Point(m_meters_over_horizon, best_b_line.y(m_meters_over_horizon)), m_line_step);
    
    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "Yellow line: y = %fx + %f", best_y_line.m, best_y_line.c);
        RCLCPP_INFO(get_logger(), "Blue line: y = %fx + %f", best_b_line.m, best_b_line.c);
    }
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