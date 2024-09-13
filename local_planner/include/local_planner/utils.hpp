#ifndef LOCAL_PLANNER_UTILS_HPP
#define LOCAL_PLANNER_UTILS_HPP

#include "local_planner/generic_planner.hpp"
#include "local_planner/skidpad.hpp"
#include "local_planner/acceleration.hpp"
#include "local_planner/autocross.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace local_planning {

namespace utils {

std::shared_ptr<GenericPlanner> create_planner();
std::vector<Point> discretize_line(double m, double q, double c, double x1, double x2, double step);
std::vector<Point> discretize_circle(Point center, double radius, double step);

} // namespace utils
} // namespace local_planning

#endif // LOCAL_PLANNER_UTILS_HPP