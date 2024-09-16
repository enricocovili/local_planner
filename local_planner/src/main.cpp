#include <typeinfo>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "local_planner/generic_planner.hpp"
#include "local_planner/acceleration.hpp"
#include "local_planner/skidpad.hpp"
#include "local_planner/autocross.hpp"



std::shared_ptr<local_planning::GenericPlanner> create_planner(std::string event_type)
{
    std::shared_ptr<local_planning::GenericPlanner> planner;

    if (event_type.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("main"), "No event type specified, loading from config file");
      std::string config_file = ament_index_cpp::get_package_share_directory("local_planner") + std::string("/config/local_planner.yaml");
      YAML::Node config = YAML::LoadFile(config_file.c_str());
      event_type = config["local_planner"]["ros__parameters"]["generic"]["event_type"].as<std::string>();
    }

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
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid event type: %s", event_type.c_str());
        rclcpp::shutdown();
        throw;
    }

    planner->init();

    return planner;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {

    auto launcher = rclcpp::Node("__event_loader");

    std::string event_type;
    launcher.declare_parameter<std::string>("event_type", "");
    launcher.get_parameter("event_type", event_type);

    auto planner = create_planner(event_type);

    rclcpp::spin(planner);
    rclcpp::shutdown();

  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Terminating due to exception of type '%s' - what(): %s", typeid(e).name(), e.what());
    rclcpp::shutdown();
    throw;
  }

  return 0;
}