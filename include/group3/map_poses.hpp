#include <iostream>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

std::map<int, geometry_msgs::msg::Pose> define_poses();
std::map<int, geometry_msgs::msg::Pose> define_tray_poses();