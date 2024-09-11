#include <local_planner/utils.hpp>

namespace local_planning {

namespace utils {

std::shared_ptr<GenericPlanner> create_planner() 
{
    // read using yaml-cpp
    std::shared_ptr<GenericPlanner> planner;

    std::string config_file = ament_index_cpp::get_package_share_directory("local_planner") + std::string("/config/local_planner.yaml");
    YAML::Node config = YAML::LoadFile(config_file.c_str());
    std::string event_type = config["local_planner"]["ros__parameters"]["generic"]["event_type"].as<std::string>();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Instantiating planner: %s", event_type.c_str());

    if (event_type == "acceleration")
    {
        planner = std::make_shared<AccelerationPlanner>();
    }
    else if (event_type == "skidpad")
    {
        planner = std::make_shared<SkidpadPlanner>();
    }
    else if (event_type == "autocross")
    {
        planner = std::make_shared<AutocrossPlanner>();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid event type: %s", event_type);
        rclcpp::shutdown();
        throw;
    }

    return planner;
}

} // namespace utils
} // namespace local_planning
 
