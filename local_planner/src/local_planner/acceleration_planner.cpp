#include "local_planner/acceleration_planner.hpp"

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

void AccelerationPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones) const
{
    // Do something with the cones
}

} // namespace local_planning