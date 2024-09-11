#ifndef ACCELLARATION_LOCAL_PLANNER_HPP
#define ACCELLARATION_LOCAL_PLANNER_HPP

#include <local_planner/generic_planner.hpp>


namespace local_planning {

class AccelerationPlanner : public GenericPlanner
{
private:
    double m_meters_over_horizon;
public:
    AccelerationPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) const override;
    void load_params() override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP