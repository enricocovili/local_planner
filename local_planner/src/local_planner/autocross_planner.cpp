#include "local_planner/autocross_planner.hpp"

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

void AutocrossPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones) const
{
    
}

} // namespace local_plannings