#ifndef LOCAL_PLANNER_UTILS_HPP
#define LOCAL_PLANNER_UTILS_HPP

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

#include "local_planner/geometry.hpp"

namespace local_planning {

namespace utils {

std::vector<Point> discretize_line(Line line, Point start, Point end, double step);
std::vector<Point> discretize_circle(Circle circle, double step);

} // namespace utils
} // namespace local_planning

#endif // LOCAL_PLANNER_UTILS_HPP