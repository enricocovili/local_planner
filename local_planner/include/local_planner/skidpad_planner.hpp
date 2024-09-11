#ifndef SKIDPAD_LOCAL_PLANNER_HPP
#define SKIDPAD_LOCAL_PLANNER_HPP

#include <local_planner/generic_planner.hpp>


namespace local_planning {

class SkidpadPlanner : public GenericPlanner
{
private:
    double m_center_x;
    double m_center_y;
    double m_inner_radius;
    double m_max_distance_from_center;
    double m_radius_offset;
    int m_circle_points;
    int m_line_points;
    double m_end_x;
    int m_minimum_cones;
public:
    SkidpadPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) const override;
    void load_params() override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP