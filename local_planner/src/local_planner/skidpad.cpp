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
    declare_parameter("skidpad.max_distance_from_center", 0.0);
    declare_parameter("skidpad.radius_offset", 0.0);
    declare_parameter("skidpad.end_x", 100.0);
    declare_parameter("skidpad.minimum_cones", 0.0);

    get_parameter("skidpad.center_x", m_center_x);
    get_parameter("skidpad.center_y", m_center_y);
    get_parameter("skidpad.inner_radius", m_inner_radius);
    get_parameter("skidpad.max_distance_from_center", m_max_distance_from_center);
    get_parameter("skidpad.radius_offset", m_radius_offset);
    get_parameter("skidpad.end_x", m_end_x);
    get_parameter("skidpad.minimum_cones", m_minimum_cones);
}

void SkidpadPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    return;
}

std::array<std::vector<Point>, 2> SkidpadPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;
    return borders;
}

std::vector<Point> SkidpadPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    return center_line;
}

} // namespace local_plannings