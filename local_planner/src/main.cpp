#include <typeinfo>

#include "local_planner/utils.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    // auto planner = local_planning::utils::create_planner();
    auto planner = std::make_shared<local_planning::GenericPlanner>();
    rclcpp::spin(planner);
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Terminating due to exception of type '%s' - what(): %s", typeid(e).name(), e.what());
    rclcpp::shutdown();
    throw;
  }

  return 0;
}