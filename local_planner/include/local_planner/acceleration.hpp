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
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) override;
    void load_params() override;
    std::array<std::vector<Point>, 2> generate_borders(std::vector<Point>, std::vector<Point>) override;
    std::vector <Point> generate_center_line(std::array<std::vector<Point>, 2>) override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP