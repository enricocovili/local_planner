#include "local_planner/generic_planner.hpp"

using Point = local_planning::Point;

namespace local_planning {

GenericPlanner::GenericPlanner() : rclcpp::Node("local_planner")
{
    m_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&GenericPlanner::init, this));
}

void GenericPlanner::init()
{
    load_params();
    create_connections();
    m_timer->cancel();
}

void GenericPlanner::load_params()
{
    // TOPICS
    declare_parameter<std::string>("generic.topics.publishers.borders", "");
    declare_parameter<std::string>("generic.topics.publishers.center_line", "");
    declare_parameter<std::string>("generic.topics.publishers.borders_completed", "");
    declare_parameter<std::string>("generic.topics.publishers.center_line_completed", "");
    
    declare_parameter<std::string>("generic.topics.subscribers.odometry", "");
    declare_parameter<std::string>("generic.topics.subscribers.race_status", "");
    declare_parameter<std::string>("generic.topics.subscribers.slam_cones", "");

    get_parameter("generic.topics.publishers.borders", this->m_borders_topic);
    get_parameter("generic.topics.publishers.center_line", m_centerLine_topic);
    get_parameter("generic.topics.publishers.borders_completed", m_borders_completed_topic);
    get_parameter("generic.topics.publishers.center_line_completed", m_centerLine_completed_topic);
    
    get_parameter("generic.topics.subscribers.odometry", m_odometry_topic);
    get_parameter("generic.topics.subscribers.race_status", m_race_status_topic);
    get_parameter("generic.topics.subscribers.slam_cones", m_slam_cones_topic);

    // OTHER PARAMETERS
    declare_parameter<double>("generic.line_step", 0.1);
    declare_parameter<double>("generic.circle_step", 0.1);
    declare_parameter<bool>("generic.debug", false);

    get_parameter("generic.line_step", m_line_step);
    get_parameter("generic.circle_step", m_circle_step);
    get_parameter("generic.debug", m_debug);

    if (m_debug)
    {
        RCLCPP_INFO(m_logger, "Generic parameters:");
        RCLCPP_INFO(m_logger, "Borders topic: %s", m_borders_topic.c_str());
        RCLCPP_INFO(m_logger, "Center Line topic: %s", m_centerLine_topic.c_str());
        RCLCPP_INFO(m_logger, "Borders completed topic: %s", m_borders_completed_topic.c_str());
        RCLCPP_INFO(m_logger, "Center Line completed topic: %s", m_centerLine_completed_topic.c_str());
        RCLCPP_INFO(m_logger, "Odometry topic: %s", m_odometry_topic.c_str());
        RCLCPP_INFO(m_logger, "Race Status topic: %s", m_race_status_topic.c_str());
        RCLCPP_INFO(m_logger, "SLAM Cones topic: %s", m_slam_cones_topic.c_str());
    }
}

void GenericPlanner::create_connections()
{
    // publishers
    m_borders_pub = this->create_publisher<mmr_base::msg::MarkerArray>(m_borders_topic, 10);
    m_centerLine_pub = this->create_publisher<mmr_base::msg::Marker>(m_centerLine_topic, 10);
    m_borders_completed_pub = this->create_publisher<mmr_base::msg::MarkerArray>(m_borders_completed_topic, 10);
    m_centerLine_completed_pub = this->create_publisher<mmr_base::msg::Marker>(m_centerLine_completed_topic, 10);

    // subscribers
    auto odometry_sub = create_subscription<nav_msgs::msg::Odometry>(m_odometry_topic, 10, std::bind(&GenericPlanner::odometry_cb, this, std::placeholders::_1));
    auto race_status_sub = create_subscription<mmr_base::msg::RaceStatus>(m_race_status_topic, 10, std::bind(&GenericPlanner::race_status_cb, this, std::placeholders::_1));
    auto slam_cones_sub = create_subscription<mmr_base::msg::Marker>(m_slam_cones_topic, 10, std::bind(&GenericPlanner::slam_cones_cb, this, std::placeholders::_1));
}

void GenericPlanner::publish_borders(std::array<std::vector<Point>, 2> borders)
{
    mmr_base::msg::MarkerArray msg;

    for (const auto i : {YELLOW, BLUE})
    {
        mmr_base::msg::Marker marker;
        marker.id = i;
        marker.ns = "border";
        marker.header.frame_id = "track";
        marker.header.stamp = get_clock()->now();
        marker.header.stamp.sec = get_clock()->now().nanoseconds() / static_cast<long int>(1e9);
        marker.header.stamp.nanosec = get_clock()->now().nanoseconds() % static_cast<long int>(1e9);
        marker.type = mmr_base::msg::Marker::LINE_STRIP;
        marker.action = mmr_base::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.color.a = 1.0;
        if (i == YELLOW)
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        for (auto point : borders[i])
        {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            marker.points.push_back(p);
        }

        msg.markers.push_back(marker);
    }

    m_borders_pub->publish(msg);
}

void GenericPlanner::publish_center_line(std::vector<Point> center_line)
{
    mmr_base::msg::Marker msg;
	msg.id = 0;
	msg.ns = "centerLine";
	msg.header.frame_id = "track";
	msg.header.stamp = get_clock()->now();
	msg.header.stamp.sec = get_clock()->now().nanoseconds() / static_cast<long int>(1e9);
	msg.header.stamp.nanosec = get_clock()->now().nanoseconds() % static_cast<long int>(1e9);
	msg.type = mmr_base::msg::Marker::LINE_STRIP;
	msg.action = mmr_base::msg::Marker::ADD;
	msg.scale.x = 0.1;
	msg.scale.y = 0.1;
	msg.scale.z = 0.1;
	msg.color.a = 1.0;
	msg.color.r = 1.0;
	msg.color.g = 1.0;
	msg.color.b = 1.0;
	msg.pose.orientation.w = 1.0;
	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 0.0;
	msg.pose.orientation.z = 0.0;

    for (auto point : center_line)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        msg.points.push_back(p);
    }

    m_centerLine_pub->publish(msg);
}

}// namespace local_planner