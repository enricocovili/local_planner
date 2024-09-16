#ifndef AUTOCROSS_LOCAL_PLANNER_HPP
#define AUTOCROSS_LOCAL_PLANNER_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <local_planner/generic_planner.hpp>


namespace local_planning {

using namespace geometry;

class AutocrossPlanner : public GenericPlanner
{
private:
    double m_search_angle;
    double m_search_distance;    
public:
    AutocrossPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) override;
    void load_params();
    std::array<std::vector<Point>, 2> generate_borders(std::vector<Point>, std::vector<Point>) override;
    std::vector <Point> generate_center_line(std::array<std::vector<Point>, 2>) override;

    double get_car_direction();
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP