#include "local_planner/autocross.hpp"

namespace local_planning {

AutocrossPlanner::AutocrossPlanner() : GenericPlanner() // Call the constructor of the parent class
{
    load_params();
}

void AutocrossPlanner::load_params()
{
    declare_parameter("autocross.search_angle", deg_to_rad(90));
    declare_parameter("autocross.search_distance", 5.0);

    get_parameter("autocross.search_angle", m_search_angle);
    get_parameter("autocross.search_distance", m_search_distance);

    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "Autocross parameters:");
        RCLCPP_INFO(get_logger(), "Search angle: %f", m_search_angle);
        RCLCPP_INFO(get_logger(), "Search distance: %f", m_search_distance);
    }
}

void AutocrossPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    if (m_idle || m_odometry == nullptr)
    {
        return;
    }
    std::array<std::vector<Point>, 2> cones;
    for (size_t i = 0; i < slam_cones->points.size(); i++)
    {
        if (slam_cones->colors[i].b > 0.9f)
        {
            cones[BLUE].push_back(Point(slam_cones->points[i]));
        }
        else
        {
            cones[YELLOW].push_back(Point(slam_cones->points[i]));
        }
    }

    // std::array<std::vector<Point>, 2> borders = generate_borders(cones[YELLOW], cones[BLUE]);

    std::vector<Point> center_line = generate_center_line(cones);

    // publish_borders(cones);
    publish_center_line(center_line);
}

double AutocrossPlanner::get_car_direction()
{
    tf2::Quaternion quat;
    tf2::fromMsg(m_odometry->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

std::array<std::vector<Point>, 2> AutocrossPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;
    return borders;
}

std::vector<Point> AutocrossPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    double curr_direction = get_car_direction();
    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "current yaw: %f", curr_direction);
    }
    // project a point 2 meters in front of the car
    Point car_position(m_odometry->pose.pose.position);
    Point front_point(car_position.x + 5 * cos(curr_direction), car_position.y + 5 * sin(curr_direction));
    center_line.push_back(m_odometry->pose.pose.position);
    center_line.push_back(front_point);
    return center_line;
}

} // namespace local_plannings