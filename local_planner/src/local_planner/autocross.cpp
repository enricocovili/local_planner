#include "local_planner/autocross.hpp"

namespace local_planning {

AutocrossPlanner::AutocrossPlanner() : GenericPlanner() // Call the constructor of the parent class
{
    load_params();
}

void AutocrossPlanner::load_params()
{
    declare_parameter("autocross.todo", "");
    get_parameter("autocross.todo", m_todo);
}

void AutocrossPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    return;
}

std::array<std::vector<Point>, 2> AutocrossPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;
    return borders;
}

std::vector<Point> AutocrossPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    return center_line;
}

} // namespace local_plannings