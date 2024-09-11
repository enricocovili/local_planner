#include "local_planner/generic_planner.hpp"

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
    declare_parameter("generic.topics.publishers.borders", "");
    declare_parameter<std::string>("generic.topics.publishers.center_line", "");
    declare_parameter<std::string>("generic.topics.publishers.borders_completed", "");
    declare_parameter<std::string>("generic.topics.publishers.center_line_completed", "");
    
    declare_parameter<std::string>("generic.topics.subscribers.odometry", "");
    declare_parameter<std::string>("generic.topics.subscribers.race_status", "");
    declare_parameter<std::string>("generic.topics.subscribers.slam_cones", "");

    // Get parameters
    get_parameter("generic.topics.publishers.borders", this->m_borders_topic);
    get_parameter("generic.topics.publishers.center_line", m_centerLine_topic);
    get_parameter("generic.topics.publishers.borders_completed", m_borders_completed_topic);
    get_parameter("generic.topics.publishers.center_line_completed", m_centerLine_completed_topic);
    
    get_parameter("generic.topics.subscribers.odometry", m_odometry_topic);
    get_parameter("generic.topics.subscribers.race_status", m_race_status_topic);
    get_parameter("generic.topics.subscribers.slam_cones", m_slam_cones_topic);

    // print parameters
    RCLCPP_INFO(m_logger, "Generic Planner parameters:");
    RCLCPP_INFO(m_logger, "Borders topic: %s", m_borders_topic.c_str());
    RCLCPP_INFO(m_logger, "Center Line topic: %s", m_centerLine_topic.c_str());
    RCLCPP_INFO(m_logger, "Borders completed topic: %s", m_borders_completed_topic.c_str());
    RCLCPP_INFO(m_logger, "Center Line completed topic: %s", m_centerLine_completed_topic.c_str());
    RCLCPP_INFO(m_logger, "Odometry topic: %s", m_odometry_topic.c_str());
    RCLCPP_INFO(m_logger, "Race Status topic: %s", m_race_status_topic.c_str());
    RCLCPP_INFO(m_logger, "SLAM Cones topic: %s", m_slam_cones_topic.c_str());
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
}// namespace local_planner