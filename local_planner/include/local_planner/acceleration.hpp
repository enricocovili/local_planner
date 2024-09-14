#ifndef ACCELLARATION_LOCAL_PLANNER_HPP
#define ACCELLARATION_LOCAL_PLANNER_HPP

#include <local_planner/generic_planner.hpp>
#include <local_planner/utils.hpp>

namespace local_planning {

class AccelerationPlanner : public GenericPlanner
{
private:
    double m_meters_over_horizon;
    double m_safe_slope;
public:
    AccelerationPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) override;
    void load_params();
    std::array<std::vector<Point>, 2> generate_borders(std::vector<Point>, std::vector<Point>) override;
    std::vector <Point> generate_center_line(std::array<std::vector<Point>, 2>) override;

    Line best_fit_line(std::vector<Point> points);
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP