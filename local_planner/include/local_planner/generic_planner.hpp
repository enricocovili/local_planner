#ifndef GENERIC_PLANNER_HPP
#define GENERIC_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mmr_base/msg/marker_array.hpp>
#include <mmr_base/msg/marker.hpp>
#include "mmr_base/msg/race_status.hpp"

#include "local_planner/geometry.hpp"

namespace local_planning {

using namespace geometry;

class GenericPlanner : public rclcpp::Node
{
private:
    std::string m_event_type;
    std::string m_borders_topic;
    std::string m_centerLine_topic;
    std::string m_borders_completed_topic;
    std::string m_centerLine_completed_topic;
    std::string m_race_status_topic;
    std::string m_odometry_topic;
    std::string m_slam_cones_topic;

    rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr m_borders_pub;
    rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr m_centerLine_pub;
    rclcpp::Publisher<mmr_base::msg::MarkerArray>::SharedPtr m_borders_completed_pub;
    rclcpp::Publisher<mmr_base::msg::Marker>::SharedPtr m_centerLine_completed_pub;

    rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr m_race_status_sub;
    mmr_base::msg::RaceStatus::SharedPtr m_race_status;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_sub;
    nav_msgs::msg::Odometry::SharedPtr m_odometry;
    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr m_slam_cones_sub;

    rclcpp::Logger m_logger = this->get_logger();
    rclcpp::TimerBase::SharedPtr m_timer;

protected:
    double m_line_step;
    double m_circle_step;
    
    bool m_debug;
    bool m_idle;

public:
    GenericPlanner();
    virtual ~GenericPlanner() = default;
    void init();
    void load_generic_params();
    void create_connections(); // setup subs, pubs and callbacks
    virtual void load_params() = 0;

    void race_status_cb(mmr_base::msg::RaceStatus::SharedPtr race_status) {m_race_status = race_status;}
    void odometry_cb(nav_msgs::msg::Odometry::SharedPtr odometry) {m_odometry = odometry;}

    void publish_borders(std::array<std::vector<Point>, 2>, bool=false);
    void publish_center_line(std::vector<Point>, bool=false);
    void publish_borders_completed(std::array<std::vector<Point>, 2>);
    void publish_center_line_completed(std::vector<Point>);

    virtual void slam_cones_cb(mmr_base::msg::Marker::SharedPtr) {};
    virtual std::array<std::vector<Point>, 2> generate_borders(std::vector<Point>, std::vector<Point>) {};
    virtual std::vector<Point> generate_center_line(std::array<std::vector<Point>, 2>) {};
};

} // namespace local_planning


#endif // PLANNER_HPP