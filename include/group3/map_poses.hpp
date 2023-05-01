/**
 * @copyright Copyright (c) 2023
 * @file map_poses.hpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Header file for map_poses.cpp
 * @version 0.2
 * @date 2023-03-04
 * 
 * 
 */
#include <iostream>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

std::map<int, geometry_msgs::msg::Pose> define_poses();
std::map<int, geometry_msgs::msg::Pose> define_tray_poses();