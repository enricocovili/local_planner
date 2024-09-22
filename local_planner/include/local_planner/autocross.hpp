#ifndef AUTOCROSS_LOCAL_PLANNER_HPP
#define AUTOCROSS_LOCAL_PLANNER_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <local_planner/generic_planner.hpp>


namespace local_planning {

using namespace geometry;

class AutocrossPlanner : public GenericPlanner
{
private:
    rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr m_debug_search_area_pub;

    double m_search_angle;
    double m_search_distance;
    double m_max_odom_distance;

    std::string m_debug_search_area_topic;

    Point get_front_point(Point, double, std::vector<Point> points, double search_angle);
    void debug_search_area(Point origin, double direction);
    double get_car_direction();

public:
    AutocrossPlanner();
    void slam_cones_cb(mmr_base::msg::Marker::SharedPtr cones) override;
    void load_params();
    std::array<std::vector<Point>, 2> generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones) override;
    std::vector <Point> generate_center_line(std::array<std::vector<Point>, 2> borders) override;
};
    
} // namespace local_plannings

#endif // ACCERLATION_LOCAL_PLANNER_HPP