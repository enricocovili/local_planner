#include "local_planner/autocross.hpp"

namespace local_planning {

AutocrossPlanner::AutocrossPlanner() : GenericPlanner()
{
    load_params();
}

void AutocrossPlanner::load_params()
{
    declare_parameter("autocross.search_angle", deg_to_rad(90));
    declare_parameter("autocross.search_distance", 5.0);
    declare_parameter("autocross.max_odom_distance", 15.0);
    declare_parameter("autocross.debug_search_area_topic", "/planning/debug/search_area");

    get_parameter("autocross.search_angle", m_search_angle);
    get_parameter("autocross.search_distance", m_search_distance);
    get_parameter("autocross.max_odom_distance", m_max_odom_distance);
    get_parameter("autocross.debug_search_area_topic", m_debug_search_area_topic);

    if (m_debug)
    {
        RCLCPP_INFO(get_logger(), "Autocross parameters:");
        RCLCPP_INFO(get_logger(), "Search angle: %f°", m_search_angle);
        RCLCPP_INFO(get_logger(), "Search distance: %f", m_search_distance);
        RCLCPP_INFO(get_logger(), "Max odom distance: %f", m_max_odom_distance);
        RCLCPP_INFO(get_logger(), "Debug search area topic: %s", m_debug_search_area_topic.c_str());
    }

    m_search_angle = deg_to_rad(m_search_angle);
    m_debug_search_area_pub = create_publisher<mmr_base::msg::Marker>(m_debug_search_area_topic, 10);

}

void AutocrossPlanner::slam_cones_cb(mmr_base::msg::Marker::SharedPtr slam_cones)
{
    if (m_idle || m_odometry == nullptr)
    {
        return;
    }
    std::array<std::vector<Point>, 2> cones;
    for (size_t i = 0; i < slam_cones->points.size(); i++)
    {
        if (point_distance(Point(m_odometry->pose.pose.position), slam_cones->points[i]) > m_max_odom_distance)
        {
            continue;
        }
        if (slam_cones->colors[i].b > 0.9f)
        {
            cones[BLUE].push_back(Point(slam_cones->points[i]));
        }
        else
        {
            cones[YELLOW].push_back(Point(slam_cones->points[i]));
        }
    }

    if (cones[YELLOW].size() < 2 || cones[BLUE].size() < 2)
    {
        RCLCPP_WARN(get_logger(), "Not enough cones detected");
        return;
    }

    std::vector<Point> center_line = generate_center_line(cones);

    if (center_line.size() < 2) // only the odometry point
    {
        return;
    }

    std::vector<Point> smooth_trajectory = smooth_points(center_line);
    if (smooth_trajectory.size() < 2)
    {
        RCLCPP_WARN(get_logger(), "The trajectory is too short");
        return;
    }
    publish_center_line(smooth_trajectory);
}

double AutocrossPlanner::get_car_direction()
{
    tf2::Quaternion quat;
    tf2::fromMsg(m_odometry->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

Point AutocrossPlanner::get_front_point(Point start, double current_angle, std::vector<Point> points, double search_angle)
{
    Point front_point;
    // get the closest point to the car that which is in range from -search_angle/2 to search_angle/2
    double min_distance = std::numeric_limits<double>::max();
    for (auto p: points)
    {
        double angle = atan2(p.y - start.y, p.x - start.x);
        double distance = point_distance(p, start);
        if (distance < min_distance && (fabs(angle - current_angle) < search_angle/2 || fabs(angle - current_angle) > 2 * M_PI - search_angle/2))
        {
            min_distance = distance;
            front_point = p;
        }
    }
    return front_point;
}

void AutocrossPlanner::debug_search_area(Point origin, double direction) {
    mmr_base::msg::Marker msg;
	msg.id = 0;
	msg.ns = "debug_search_area";
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

    // create a line and circle that represents the search area
    std::vector<Point> debug_area;

    // create a partial circle, from -search_angle/2 to search_angle/2 with a radius of m_search_distance around the origin
    std::vector<Point> search_circle;
    for (double angle = direction - m_search_angle/2; angle <= direction + m_search_angle/2; angle += 0.1)
    {
        search_circle.push_back(Point(origin.x + m_search_distance * cos(angle), origin.y + m_search_distance * sin(angle)));
    }

    debug_area.push_back(m_odometry->pose.pose.position);
    // debug_area.push_back(left_point);
    debug_area.insert(debug_area.end(), search_circle.begin(), search_circle.end());
    // debug_area.push_back(right_point);
    debug_area.push_back(m_odometry->pose.pose.position);

    for (size_t i = 0; i < debug_area.size(); i++)
    {
        geometry_msgs::msg::Point p;
        p.x = debug_area[i].x;
        p.y = debug_area[i].y;
        msg.points.push_back(p);
    }

    m_debug_search_area_pub->publish(msg);
}

std::array<std::vector<Point>, 2> AutocrossPlanner::generate_borders(std::vector<Point> y_cones, std::vector<Point> b_cones)
{
    std::array<std::vector<Point>, 2> borders;
    return borders;
}

std::vector<Point> AutocrossPlanner::generate_center_line(std::array<std::vector<Point>, 2> borders)
{
    std::vector<Point> center_line;
    center_line.push_back(Point(m_odometry->pose.pose.position));
    // get the front point of the car
    Point front_point = m_odometry->pose.pose.position;
    double current_angle = get_car_direction();
    Point y_point, b_point;
    debug_search_area(front_point, current_angle);
    for (size_t i = 0; i < 10 && path_length(center_line) <= 10.0; ++i)
    {
        y_point = get_front_point(front_point, current_angle, borders[YELLOW], m_search_angle);
        b_point = get_front_point(front_point, current_angle, borders[BLUE], m_search_angle);
        if (y_point == Point() || b_point == Point())
        {
            if (front_point == m_odometry->pose.pose.position)
            {
                RCLCPP_WARN(get_logger(), "No point in front of the car was found");
            }
            return center_line;
        }
        borders[YELLOW].erase(std::remove(borders[YELLOW].begin(), borders[YELLOW].end(), y_point), borders[YELLOW].end());
        borders[BLUE].erase(std::remove(borders[BLUE].begin(), borders[BLUE].end(), b_point), borders[BLUE].end());
        Point next_front_point = Point((y_point.x + b_point.x) / 2, (y_point.y + b_point.y) / 2);
        current_angle = atan2(next_front_point.y - front_point.y, next_front_point.x - front_point.x);
        front_point = next_front_point;
        center_line.push_back(front_point);
    }

    // SAFETY CHECK: sometimes, points a little behind of the car are used as front points and the trajectory is generated in the wrong direction
    // so, if the first point (after the odometry) is behind the odometry, the trajectory is discarded
    // this is accomplished checking if the first point after odometry is in the search_area
    double check_angle = atan2(center_line[1].y - center_line[0].y, center_line[1].x - center_line[0].x);
    if (fabs(check_angle - get_car_direction()) > m_search_angle/2)
    {
        RCLCPP_WARN(get_logger(), "The trajectory is in the wrong direction, discarding it");
        center_line.clear();
        center_line.push_back(Point(m_odometry->pose.pose.position));
    }

    return center_line;
}

} // namespace local_planning