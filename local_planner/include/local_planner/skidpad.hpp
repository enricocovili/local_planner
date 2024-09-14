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
    double m_end_x;
    int m_minimum_cones;
public:
    SkidpadPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) override;
    void load_params();
    std::array<std::vector<Point>, 2> generate_borders(std::vector<Point>, std::vector<Point>) override;
    std::vector <Point> generate_center_line(std::array<std::vector<Point>, 2>) override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP