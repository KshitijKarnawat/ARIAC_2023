/**
 * @file floor_robot.hpp
 * @author Sanchit Kedia (sanchit@terpmail.umd.edu)
 * @author Adarsh Malapaka (amalapak@terpmail.umd.edu)
 * @author Tanmay Haldankar (tanmayh@terpmail.umd.edu)
 * @author Sahruday Patti (sahruday@umd.edu)
 * @author Kshitij Karnawat (kshitij@umd.edu)
 * @brief Implementation of the FloorRobot class for ARIAC 2023 (Group 3) 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <array>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unistd.h>

#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/competition_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>

#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include "group3/srv/floor_change_gripper.hpp"
#include "group3/srv/floor_pick_tray.hpp"
#include "group3/srv/floor_pick_part_bin.hpp"
#include "group3/srv/floor_place_part.hpp"

class FloorRobot : public rclcpp::Node
{
public:
  FloorRobot();

  ~FloorRobot();

  bool FloorRobotSetGripperState(bool enable);
  // Service Callbacks
  void FloorRobotMoveHome(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
    
  void FloorRobotChangeGripper(
    group3::srv::FloorChangeGripper::Request::SharedPtr req,
    group3::srv::FloorChangeGripper::Response::SharedPtr res);

  void FloorRobotPickandPlaceTray(
    group3::srv::FloorPickTray::Request::SharedPtr req,
    group3::srv::FloorPickTray::Response::SharedPtr res);

  void FloorRobotPickBinPart(
    group3::srv::FloorPickPartBin::Request::SharedPtr req,
    group3::srv::FloorPickPartBin::Response::SharedPtr res);

  void FloorRobotPlacePartOnKitTray(
    group3::srv::FloorPlacePart::Request::SharedPtr req,
    group3::srv::FloorPlacePart::Response::SharedPtr res);

private:

  bool FloorRobotMovetoTarget();
  bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
  void FloorRobotWaitForAttach(double timeout,std::vector<geometry_msgs::msg::Pose> waypoints);

  geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

  void LogPose(geometry_msgs::msg::Pose p);
  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
  geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
  geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
  double GetYaw(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

  void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
  void AddModelsToPlanningScene();

  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  
  // MoveIt Interfaces 
  moveit::planning_interface::MoveGroupInterface floor_robot_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr floor_robot_move_home_srv_;
  rclcpp::Service<group3::srv::FloorChangeGripper>::SharedPtr floor_robot_change_gripper_srv_;
  rclcpp::Service<group3::srv::FloorPickTray>::SharedPtr floor_pick_place_tray_srv_;
  rclcpp::Service<group3::srv::FloorPickPartBin>::SharedPtr floor_pick_part_bin_srv_;
  rclcpp::Service<group3::srv::FloorPlacePart>::SharedPtr floor_place_part_srv_;

  // Gripper State Callback
  void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

  // ARIAC Services
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;

  // Gripper State
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
  ariac_msgs::msg::Part floor_robot_attached_part_;
  ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
  ariac_msgs::msg::Part ceiling_robot_attached_part_;

  // Constants
  double kit_tray_thickness_ = 0.01;
  double drop_height_ = 0.002;
  double pick_offset_ = 0.003;
  double battery_grip_offset_ = -0.05;

  std::map<int, std::string> part_types_ = {
      {ariac_msgs::msg::Part::BATTERY, "battery"},
      {ariac_msgs::msg::Part::PUMP, "pump"},
      {ariac_msgs::msg::Part::REGULATOR, "regulator"},
      {ariac_msgs::msg::Part::SENSOR, "sensor"}};

  std::map<int, std::string> part_colors_ = {
      {ariac_msgs::msg::Part::RED, "red"},
      {ariac_msgs::msg::Part::BLUE, "blue"},
      {ariac_msgs::msg::Part::GREEN, "green"},
      {ariac_msgs::msg::Part::ORANGE, "orange"},
      {ariac_msgs::msg::Part::PURPLE, "purple"},
  };

  // Part heights
  std::map<int, double> part_heights_ = {
      {ariac_msgs::msg::Part::BATTERY, 0.04},
      {ariac_msgs::msg::Part::PUMP, 0.12},
      {ariac_msgs::msg::Part::REGULATOR, 0.07},
      {ariac_msgs::msg::Part::SENSOR, 0.07}};

  // Quadrant Offsets
  std::map<int, std::pair<double, double>> quad_offsets_ = {
      {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
  };

  std::map<std::string, double> rail_positions_ = {
      {"agv1", -4.5},
      {"agv2", -1.2},
      {"agv3", 1.2},
      {"agv4", 4.5},
      {"left_bins", 3},
      {"right_bins", -3}};

  // Joint value targets for kitting stations
  std::map<std::string, double> floor_kts1_js_ = {
      {"linear_actuator_joint", 4.0},
      {"floor_shoulder_pan_joint", 1.57},
      {"floor_shoulder_lift_joint", -1.57},
      {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},
      {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};

  std::map<std::string, double> floor_kts2_js_ = {
      {"linear_actuator_joint", -4.0},
      {"floor_shoulder_pan_joint", -1.57},
      {"floor_shoulder_lift_joint", -1.57},
      {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},
      {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};

  std::map<int, std::vector<float>> bin_quadrant_positions_ = {
      {1, std::vector<float>{0.1, -0.1, 0.05}},
  };

  std::map<int, std::vector<float>> tray_positions_ = {
      {0, std::vector<float>{-0.87, -5.84, 0.734990}},
      {1, std::vector<float>{-1.3, -5.84, 0.734990}},
      {2, std::vector<float>{-1.730, -5.84, 0.734990}},
      {3, std::vector<float>{-1.730, 5.84, 0.734990}},
      {4, std::vector<float>{-1.3, 5.84, 0.734990}},
      {5, std::vector<float>{-0.87, 5.84, 0.734990}},
  };
};