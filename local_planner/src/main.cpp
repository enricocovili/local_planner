#include <typeinfo>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "local_planner/generic_planner.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    // Load and parse the YAML file
    std::string config_path = ament_index_cpp::get_package_share_directory("local_planner") + "/config/local_planner.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    auto planner = std::make_shared<local_planning::GenericPlanner>();
    
    std::string event_type = config["event_type"].as<std::string>();
    if (event_type == "acceleration") {
      planner = std::make_shared<local_planning::AccelerationPlanner>();
    } else if (event_type == "skidpad") {
      planner = std::make_shared<local_planning::SkidpadPlanner>();
    } else if (event_type == "autocross") {
      planner = std::make_shared<local_planning::AutocrossPlanner>();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid event type: %s", event_type.c_str());
      rclcpp::shutdown();
      throw;
    }

    planner.init();
    rclcpp::spin(planner);
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Terminating due to exception of type '%s' - what(): %s", typeid(e).name(), e.what());
    rclcpp::shutdown();
    throw;
  }

  return 0;
}