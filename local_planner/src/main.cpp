#include <typeinfo>

#include "local_planner/generic_planner.hpp"
#include "local_planner/acceleration.hpp"
#include "local_planner/skidpad.hpp"
#include "local_planner/autocross.hpp"

std::shared_ptr<local_planning::GenericPlanner> create_planner()
{
    std::shared_ptr<local_planning::GenericPlanner> planner;

    std::string config_file = ament_index_cpp::get_package_share_directory("local_planner") + std::string("/config/local_planner.yaml");
    YAML::Node config = YAML::LoadFile(config_file.c_str());
    std::string event_type = config["local_planner"]["ros__parameters"]["generic"]["event_type"].as<std::string>();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Instantiating planner: %s", event_type.c_str());

    if (event_type == "acceleration")
    {
        planner = std::make_shared<local_planning::AccelerationPlanner>();
    }
    else if (event_type == "skidpad")
    {
        planner = std::make_shared<local_planning::SkidpadPlanner>();
    }
    else if (event_type == "autocross")
    {
        planner = std::make_shared<local_planning::AutocrossPlanner>();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid event type: %s", event_type);
        rclcpp::shutdown();
        throw;
    }

    return planner;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto planner = create_planner();
    // auto planner = std::make_shared<local_planning::GenericPlanner>();
    rclcpp::spin(planner);
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Terminating due to exception of type '%s' - what(): %s", typeid(e).name(), e.what());
    rclcpp::shutdown();
    throw;
  }

  return 0;
}